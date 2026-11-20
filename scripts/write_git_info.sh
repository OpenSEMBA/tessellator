#!/usr/bin/env sh

set -eu

outfile=${1:-git_info.txt}

if command -v git >/dev/null 2>&1 && [ -e .git ]; then
    commit=$(git rev-parse --short HEAD 2>/dev/null || printf '%s' UNKNOWN)
else
    commit=UNKNOWN
fi

printf 'GIT_COMMIT="%s"\n' "$commit" > "$outfile"
printf 'Wrote commit info to %s (commit: %s)\n' "$outfile" "$commit"
