#!/bin/bash
# 验证 src/ 下所有源码剥离注释后的哈希与基线一致。
# 用途:确认"只加注释"的改动没有触碰任何代码(语句/字符串/数字/缩进/行尾)。
#
# 用法:
#   bash tools/verify_comments.sh             # 验证全部文件
#   bash tools/verify_comments.sh analysis    # 只验证包含 analysis 的路径
#   bash tools/verify_comments.sh --rebuild   # 重新生成基线后验证
#
# 退出码:0 = 全部一致;1 = 存在不一致(输出差异文件列表)。
# 基线默认存放在 build/comment_baseline/(被 .gitignore 忽略,不入库)。
set -u
cd "$(dirname "$0")/.."

strip_awk="tools/strip_comments.awk"
base_dir="build/comment_baseline"

rebuild=0
filter=""
for arg in "$@"; do
    case "$arg" in
        --rebuild) rebuild=1 ;;
        *) filter="$arg" ;;
    esac
done

if [ "$rebuild" -eq 1 ] || [ ! -d "$base_dir" ]; then
    echo ">> 重新生成基线到 $base_dir/ ..."
    rm -rf "$base_dir"
    mkdir -p "$base_dir"
    for f in $(find src -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.inc' | sort); do
        rel="${f#src/}"
        mkdir -p "$base_dir/$(dirname "$rel")"
        awk -f "$strip_awk" "$f" | sha256sum | cut -d' ' -f1 > "$base_dir/$rel.sha"
    done
fi

fail=0
total=0
for f in $(find src -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.inc' | sort); do
    [ -n "$filter" ] && case "$f" in *"$filter"*) ;; *) continue;; esac
    rel="${f#src/}"
    base="$base_dir/$rel.sha"
    [ -f "$base" ] || { echo "MISSING-BASELINE: $f (先运行 --rebuild)"; fail=1; continue; }
    total=$((total+1))
    cur=$(awk -f "$strip_awk" "$f" | sha256sum | cut -d' ' -f1)
    want=$(cat "$base")
    if [ "$cur" != "$want" ]; then
        echo "DIFF: $f"
        fail=1
    fi
done
echo "检查 $total 个文件"
[ $fail -eq 0 ] && echo "OK: 全部一致(零代码改动)" || echo "FAIL: 以上文件存在非注释改动"
exit $fail
