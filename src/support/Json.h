#pragma once

#include <nlohmann/json.hpp>

#include <charconv>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

namespace oap {

class JsonArray;
class JsonObject;

// JSON field access is deliberately type-strict. Missing and explicit null
// remain distinct so older project schemas can be validated and migrated.
class JsonValue {
public:
    using Storage = nlohmann::json;
    enum NullType { Null };

    JsonValue() = default;
    JsonValue(NullType) {}
    JsonValue(const char* value) : m_value(value) {}
    JsonValue(std::string value) : m_value(std::move(value)) {}
    JsonValue(bool value) : m_value(value) {}
    template<std::integral T>
    JsonValue(T value) : m_value(value) {}
    JsonValue(double value) : m_value(value)
    {
        if (value >= -0x1p63 && value < 0x1p63 && std::trunc(value) == value)
            m_value = static_cast<std::int64_t>(value);
    }
    JsonValue(JsonArray value);
    JsonValue(JsonObject value);

    bool isUndefined() const { return m_value.is_discarded(); }
    bool isNull() const { return m_value.is_null(); }
    bool isObject() const { return m_value.is_object(); }
    bool isArray() const { return m_value.is_array(); }
    bool isString() const { return m_value.is_string(); }
    bool isBool() const { return m_value.is_boolean(); }
    bool isDouble() const { return m_value.is_number(); }
    std::string toString(std::string_view fallback = {}) const
    {
        return isString() ? m_value.get<std::string>() : std::string(fallback);
    }
    std::string scalarText() const
    {
        if (isString()) return toString();
        return (isBool() || isDouble()) ? m_value.dump() : std::string{};
    }
    double toDouble(double fallback = 0.0) const
    {
        return isDouble() ? m_value.get<double>() : fallback;
    }
    int toInt(int fallback = 0) const
    {
        const auto value = toDouble(fallback);
        return value >= std::numeric_limits<int>::min() &&
               value <= std::numeric_limits<int>::max() && std::trunc(value) == value
                   ? static_cast<int>(value) : fallback;
    }
    bool toBool(bool fallback = false) const
    {
        return isBool() ? m_value.get<bool>() : fallback;
    }
    JsonObject toObject() const;
    JsonArray toArray() const;
    bool operator==(std::string_view text) const
    {
        const auto* value = m_value.get_ptr<const std::string*>();
        return value && *value == text;
    }

    static JsonValue missing() { return fromStorage(Storage::value_t::discarded); }
    static JsonValue fromStorage(Storage value)
    {
        JsonValue result;
        result.m_value = std::move(value);
        return result;
    }

protected:
    Storage m_value;
    friend class JsonArray;
    friend class JsonObject;
    friend class JsonDocument;
};

class JsonIterator {
public:
    explicit JsonIterator(JsonValue::Storage::const_iterator iterator) : m_iterator(iterator) {}
    bool operator==(const JsonIterator&) const = default;
    JsonIterator& operator++() { ++m_iterator; return *this; }
    JsonValue operator*() const { return JsonValue::fromStorage(*m_iterator); }
    const std::string& key() const { return m_iterator.key(); }
    JsonValue value() const { return **this; }

private:
    JsonValue::Storage::const_iterator m_iterator;
};

class JsonArray {
public:
    using Storage = JsonValue::Storage;

    JsonArray() : m_value(Storage::array()) {}
    JsonArray(std::initializer_list<JsonValue> values) : JsonArray()
    {
        for (const auto& value : values) m_value.push_back(value.m_value);
    }
    explicit JsonArray(Storage value) : m_value(std::move(value)) {}

    bool empty() const { return m_value.empty(); }
    std::size_t size() const { return m_value.size(); }
    JsonIterator begin() const { return JsonIterator(m_value.begin()); }
    JsonIterator end() const { return JsonIterator(m_value.end()); }
    JsonValue at(std::size_t index) const
    {
        return index < size() ? JsonValue::fromStorage(m_value[index]) : JsonValue::missing();
    }
    JsonValue operator[](std::size_t index) const { return at(index); }
    void append(JsonValue value) { m_value.push_back(std::move(value.m_value)); }
    void replace(std::size_t index, JsonValue value) { m_value.at(index) = std::move(value.m_value); }

private:
    Storage m_value;
    friend class JsonValue;
    friend class JsonDocument;
};

class JsonObject {
public:
    using Storage = JsonValue::Storage;

    JsonObject() : m_value(Storage::object()) {}
    JsonObject(std::initializer_list<std::pair<std::string, JsonValue>> values) : JsonObject()
    {
        for (const auto& [key, value] : values) m_value[key] = value.m_value;
    }
    explicit JsonObject(Storage value) : m_value(std::move(value)) {}

    bool empty() const { return m_value.empty(); }
    std::size_t size() const { return m_value.size(); }
    bool operator==(const JsonObject&) const = default;
    bool contains(const std::string& key) const { return m_value.contains(key); }
    JsonIterator begin() const { return JsonIterator(m_value.begin()); }
    JsonIterator end() const { return JsonIterator(m_value.end()); }
    JsonValue value(const std::string& key) const
    {
        const auto found = m_value.find(key);
        return found == m_value.end() ? JsonValue::missing() : JsonValue::fromStorage(*found);
    }
    JsonValue operator[](const std::string& key) const { return value(key); }
    void insert(const std::string& key, JsonValue value)
    {
        if (value.isUndefined()) m_value.erase(key);
        else m_value[key] = std::move(value.m_value);
    }
    void remove(const std::string& key) { m_value.erase(key); }

private:
    Storage m_value;
    friend class JsonValue;
    friend class JsonDocument;
};

inline JsonValue::JsonValue(JsonArray value) : m_value(std::move(value.m_value)) {}
inline JsonValue::JsonValue(JsonObject value) : m_value(std::move(value.m_value)) {}
inline JsonObject JsonValue::toObject() const { return isObject() ? JsonObject(m_value) : JsonObject{}; }
inline JsonArray JsonValue::toArray() const { return isArray() ? JsonArray(m_value) : JsonArray{}; }

struct JsonParseError {
    std::string message;
    std::size_t offset = 0;
    explicit operator bool() const { return !message.empty(); }
};

class JsonDocument {
public:
    enum Formatting { Compact, Indented };

    JsonDocument() = default;
    explicit JsonDocument(JsonObject value) : m_value(std::move(value.m_value)) {}
    explicit JsonDocument(JsonArray value) : m_value(std::move(value.m_value)) {}

    bool isObject() const { return m_value.is_object(); }
    bool isArray() const { return m_value.is_array(); }
    JsonObject object() const { return isObject() ? JsonObject(m_value) : JsonObject{}; }
    JsonArray array() const { return isArray() ? JsonArray(m_value) : JsonArray{}; }
    std::string toJson(Formatting formatting = Indented) const
    {
        return formatting == Compact ? m_value.dump() : m_value.dump(4) + '\n';
    }

    // Persisted setup confirmations hash the shortest representation of real
    // coordinates. Use that encoding even when the JSON library prefers a
    // different (but numerically equivalent) decimal/exponent representation.
    std::string toFingerprintJson() const
    {
        std::string text;
        appendFingerprintValue(text, m_value);
        return text;
    }

    static JsonDocument fromJson(std::string_view bytes, JsonParseError* error = nullptr)
    {
        JsonDocument result;
        try {
            result.m_value = JsonValue::Storage::parse(bytes);
            if (!result.isObject() && !result.isArray() && error)
                error->message = "Expected a JSON object or array";
        } catch (const JsonValue::Storage::parse_error& failure) {
            if (error) *error = {failure.what(), failure.byte == 0 ? 0 : failure.byte - 1};
        } catch (const JsonValue::Storage::out_of_range& failure) {
            if (error) error->message = failure.what();
        }
        return result;
    }

private:
    static void appendFingerprintValue(std::string& text, const JsonValue::Storage& value)
    {
        if (value.is_object() || value.is_array()) {
            const bool object = value.is_object();
            text += object ? '{' : '[';
            for (auto item = value.begin(); item != value.end(); ++item) {
                if (item != value.begin()) text += ',';
                if (object) {
                    text += JsonValue::Storage(item.key()).dump();
                    text += ':';
                }
                appendFingerprintValue(text, *item);
            }
            text += object ? '}' : ']';
        } else if (value.is_number_float() && std::isfinite(value.get<double>())) {
            char buffer[32];
            const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value.get<double>());
            if (result.ec != std::errc{}) throw std::runtime_error("Cannot encode JSON fingerprint");
            text.append(buffer, result.ptr);
        } else {
            text += value.dump();
        }
    }

    JsonValue::Storage m_value;
};

} // namespace oap
