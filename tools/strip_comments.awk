# 从 C/C++ 源文件中剥离注释,输出"代码部分"。
# 用于验证"注释改动"没有触碰任何代码。
# 注意:字符串字面量中的 // 或 /* 会被误判(如 URL),但注释任务禁止改字符串,
# 误判只会造成基线哈希不一致的假报警,不会漏报真实的代码改动。
# 行尾保留(CRLF 的 \r 参与哈希),因此该脚本对"代码行行尾被改动"同样敏感。
BEGIN { in_block = 0 }
{
    line = $0
    out = ""
    i = 1
    n = length(line)
    while (i <= n) {
        c = substr(line, i, 1)
        if (in_block) {
            if (c == "*" && substr(line, i + 1, 1) == "/") {
                in_block = 0
                i = i + 2
            } else {
                i = i + 1
            }
            continue
        }
        if (c == "/" && substr(line, i + 1, 1) == "*") {
            in_block = 1
            i = i + 2
            continue
        }
        if (c == "/" && substr(line, i + 1, 1) == "/") {
            break   # 行注释:丢弃到行尾
        }
        out = out c
        i = i + 1
    }
    gsub(/[ \t]+/, "", out)
    if (out != "") print out
}
