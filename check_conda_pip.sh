#!/usr/bin/env bash

set +e

echo "=========================================="
echo " Conda / Python / Pip Environment Checker "
echo "=========================================="
echo

echo "===== 1. BASIC INFO ====="
echo "USER              : $USER"
echo "SHELL             : $SHELL"
echo "BASH_VERSION      : $BASH_VERSION"
echo "CONDA_PREFIX      : ${CONDA_PREFIX:-<not set>}"
echo "CONDA_DEFAULT_ENV : ${CONDA_DEFAULT_ENV:-<not set>}"
echo

echo "===== 2. PYTHON RESOLUTION ====="
echo "[command -v python]"
command -v python
echo

echo "[type -a python]"
type -a python 2>&1
echo

if command -v python >/dev/null 2>&1; then
    echo "[python version]"
    python --version 2>&1
    echo

    echo "[python realpath]"
    readlink -f "$(command -v python)" 2>/dev/null
fi
echo

echo "===== 3. PIP RESOLUTION ====="
echo "[command -v pip]"
command -v pip
echo

echo "[type -a pip]"
type -a pip 2>&1
echo

if command -v pip >/dev/null 2>&1; then
    echo "[pip -V]"
    pip -V 2>&1
    echo

    echo "[pip realpath]"
    readlink -f "$(command -v pip)" 2>/dev/null
fi
echo

echo "===== 4. PYTHON -M PIP ====="
python -m pip -V 2>&1
echo

echo "===== 5. EXPECTED CONDA BIN ====="
if [ -n "${CONDA_PREFIX:-}" ]; then
    echo "Expected bin:"
    echo "$CONDA_PREFIX/bin"
    echo

    echo "[pip files in current conda env]"
    ls -l "$CONDA_PREFIX"/bin/pip* 2>&1
    echo

    echo "[python files in current conda env]"
    ls -l "$CONDA_PREFIX"/bin/python* 2>&1
else
    echo "WARNING: CONDA_PREFIX is not set."
    echo "You probably have not activated a conda environment."
fi
echo

echo "===== 6. PATH ORDER ====="
echo "$PATH" | tr ':' '\n' | nl -ba
echo

if [ -n "${CONDA_PREFIX:-}" ]; then
    echo "Current conda bin position in PATH:"
    echo "$PATH" | tr ':' '\n' | nl -ba | grep -F "$CONDA_PREFIX/bin"
    echo
fi

echo "===== 7. DUPLICATED PATH ENTRIES ====="
echo "$PATH" | tr ':' '\n' | sort | uniq -cd | sort -nr
echo

echo "===== 8. PIP ALIAS / FUNCTION CHECK ====="
echo "[alias pip]"
alias pip 2>&1
echo

echo "[declare -f pip]"
declare -f pip 2>&1
echo

echo "[command -V pip]"
command -V pip 2>&1
echo

echo "===== 9. PYTHON ALIAS / FUNCTION CHECK ====="
echo "[alias python]"
alias python 2>&1
echo

echo "[declare -f python]"
declare -f python 2>&1
echo

echo "===== 10. SHELL CONFIG SEARCH ====="

FILES=(
    "$HOME/.bashrc"
    "$HOME/.bash_profile"
    "$HOME/.bash_login"
    "$HOME/.profile"
)

for f in "${FILES[@]}"; do
    if [ -f "$f" ]; then
        echo
        echo "---- $f ----"
        grep -nEi \
            'PATH=|conda|pip|python|alias[[:space:]]+pip|alias[[:space:]]+python' \
            "$f" 2>/dev/null
    fi
done
echo

echo "===== 11. SYSTEM SHELL CONFIG SEARCH ====="

for f in /etc/profile /etc/bash.bashrc; do
    if [ -f "$f" ]; then
        echo
        echo "---- $f ----"
        grep -nEi \
            'PATH=|conda|pip|python|alias[[:space:]]+pip|alias[[:space:]]+python' \
            "$f" 2>/dev/null
    fi
done
echo

echo "===== 12. CONDA ACTIVATE.D CHECK ====="

SEARCH_DIRS=()

if [ -n "${CONDA_PREFIX:-}" ]; then
    SEARCH_DIRS+=("$CONDA_PREFIX/etc/conda")
fi

if [ -n "${CONDA_EXE:-}" ]; then
    CONDA_BASE="$(dirname "$(dirname "$CONDA_EXE")")"
    SEARCH_DIRS+=("$CONDA_BASE/etc/conda")
fi

SEARCH_DIRS+=("$HOME/.conda")

for d in "${SEARCH_DIRS[@]}"; do
    if [ -d "$d" ]; then
        echo
        echo "---- $d ----"
        find "$d" -maxdepth 4 -type f 2>/dev/null
        echo
        grep -RniE \
            'PATH=|pip|python' \
            "$d" 2>/dev/null
    fi
done
echo

echo "===== 13. CONSISTENCY CHECK ====="

CURRENT_PYTHON="$(command -v python 2>/dev/null)"
CURRENT_PIP="$(command -v pip 2>/dev/null)"

if [ -n "${CONDA_PREFIX:-}" ]; then
    EXPECTED_PREFIX="$CONDA_PREFIX/bin"

    if [[ "$CURRENT_PYTHON" == "$EXPECTED_PREFIX/"* ]]; then
        echo "[OK] python points to current conda environment"
    else
        echo "[FAIL] python does NOT point to current conda environment"
        echo "       python = $CURRENT_PYTHON"
    fi

    if [[ "$CURRENT_PIP" == "$EXPECTED_PREFIX/"* ]]; then
        echo "[OK] pip points to current conda environment"
    else
        echo "[FAIL] pip does NOT point to current conda environment"
        echo "       pip = $CURRENT_PIP"
        echo "       expected prefix = $EXPECTED_PREFIX"
    fi
else
    echo "[WARN] No active conda environment."
fi

echo

echo "===== 14. LIKELY CAUSE ====="

if [ -n "${CONDA_PREFIX:-}" ]; then
    PIP_TYPE="$(type -t pip 2>/dev/null)"

    if [ "$PIP_TYPE" = "alias" ]; then
        echo "[CAUSE] pip is an alias."
        echo "Try:"
        echo "  unalias pip"
    elif [ "$PIP_TYPE" = "function" ]; then
        echo "[CAUSE] pip is a shell function."
        echo "Try:"
        echo "  unset -f pip"
    elif [[ "$CURRENT_PIP" != "$CONDA_PREFIX/bin/"* ]]; then
        echo "[CAUSE] PATH ordering is probably wrong."
        echo
        echo "Current pip:"
        echo "  $CURRENT_PIP"
        echo
        echo "Expected:"
        echo "  $CONDA_PREFIX/bin/pip"
        echo
        echo "Temporary fix:"
        echo '  export PATH="$CONDA_PREFIX/bin:$PATH"'
        echo "  hash -r"
    else
        echo "[OK] No obvious pip path problem detected."
    fi
fi

echo
echo "=========================================="
echo " Finished"
echo "=========================================="
