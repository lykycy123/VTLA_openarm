#!/usr/bin/env bash
set -u

timestamp="$(date +%Y%m%d_%H%M%S)"
BACKUP_ROOT="$HOME/conda_pip_repair_backup_$timestamp"
mkdir -p "$BACKUP_ROOT"

echo "============================================================"
echo " Conda / Pip Repair"
echo "============================================================"
echo "Backup directory: $BACKUP_ROOT"
echo

# ------------------------------------------------------------
# 1. Locate conda
# ------------------------------------------------------------
CONDA_EXE_PATH=""

if [ -n "${CONDA_EXE:-}" ] && [ -x "${CONDA_EXE:-}" ]; then
    CONDA_EXE_PATH="$CONDA_EXE"
fi

if [ -z "$CONDA_EXE_PATH" ]; then
    for candidate in \
        "$HOME/anaconda3/bin/conda" \
        "$HOME/miniconda3/bin/conda" \
        "$HOME/miniforge3/bin/conda" \
        "$HOME/mambaforge/bin/conda"
    do
        if [ -x "$candidate" ]; then
            CONDA_EXE_PATH="$candidate"
            break
        fi
    done
fi

if [ -z "$CONDA_EXE_PATH" ]; then
    echo "[ERROR] conda executable not found."
    exit 1
fi

CONDA_BASE="$("$CONDA_EXE_PATH" info --base 2>/dev/null)"

if [ -z "$CONDA_BASE" ] || [ ! -d "$CONDA_BASE" ]; then
    echo "[ERROR] Could not determine conda base."
    exit 1
fi

echo "[OK] Conda executable: $CONDA_EXE_PATH"
echo "[OK] Conda base      : $CONDA_BASE"
echo

# ------------------------------------------------------------
# 2. Backup shell configs
# ------------------------------------------------------------
echo "===== Backing up shell configuration ====="

for f in \
    "$HOME/.bashrc" \
    "$HOME/.profile" \
    "$HOME/.bash_profile" \
    "$HOME/.bash_login"
do
    if [ -f "$f" ]; then
        cp -a "$f" "$BACKUP_ROOT/$(basename "$f").backup"
        echo "[BACKUP] $f"
    fi
done
echo

# ------------------------------------------------------------
# 3. Move user-site pip out of the way
# ------------------------------------------------------------
echo "===== Removing user-site pip shadowing ====="

mkdir -p "$BACKUP_ROOT/user_pip"

move_if_exists() {
    src="$1"

    if [ -e "$src" ] || [ -L "$src" ]; then
        base="$(basename "$src")"
        dest="$BACKUP_ROOT/user_pip/$base"

        if [ -e "$dest" ]; then
            dest="$BACKUP_ROOT/user_pip/${base}_$RANDOM"
        fi

        mv "$src" "$dest"
        echo "[MOVED] $src"
    fi
}

move_if_exists "$HOME/.local/bin/pip"
move_if_exists "$HOME/.local/bin/pip3"

if [ -d "$HOME/.local/bin" ]; then
    while IFS= read -r -d '' f; do
        move_if_exists "$f"
    done < <(find "$HOME/.local/bin" -maxdepth 1 -type f -name 'pip3.*' -print0 2>/dev/null)
fi

# Move only pip package metadata/package directories from user site.
# Other ~/.local Python packages are untouched.
if [ -d "$HOME/.local/lib" ]; then
    while IFS= read -r -d '' py_site; do
        pyver="$(basename "$(dirname "$py_site")")"
        mkdir -p "$BACKUP_ROOT/user_pip/$pyver"

        if [ -d "$py_site/pip" ]; then
            dest="$BACKUP_ROOT/user_pip/$pyver/pip"
            [ -e "$dest" ] && dest="${dest}_$RANDOM"
            mv "$py_site/pip" "$dest"
            echo "[MOVED] $py_site/pip"
        fi

        while IFS= read -r -d '' dist; do
            dest="$BACKUP_ROOT/user_pip/$pyver/$(basename "$dist")"
            [ -e "$dest" ] && dest="${dest}_$RANDOM"
            mv "$dist" "$dest"
            echo "[MOVED] $dist"
        done < <(find "$py_site" -maxdepth 1 -type d -name 'pip-*.dist-info' -print0 2>/dev/null)

    done < <(find "$HOME/.local/lib" -maxdepth 2 -type d -name site-packages -print0 2>/dev/null)
fi
echo

# ------------------------------------------------------------
# 4. Remove all existing managed conda init blocks
# ------------------------------------------------------------
echo "===== Rebuilding ~/.bashrc conda initialization ====="

touch "$HOME/.bashrc"

python3 - "$HOME/.bashrc" <<'PY_BASHRC'
import sys
from pathlib import Path

path = Path(sys.argv[1])
lines = path.read_text(errors="replace").splitlines(keepends=True)

begin = "# >>> conda initialize >>>"
end = "# <<< conda initialize <<<"

out = []
inside = False
count = 0

for line in lines:
    s = line.strip()

    if s == begin:
        inside = True
        count += 1
        continue

    if inside:
        if s == end:
            inside = False
        continue

    if s == end:
        count += 1
        continue

    out.append(line)

path.write_text("".join(out).rstrip() + "\n")
print(f"[INFO] Removed {count} old conda init marker/block occurrence(s).")
PY_BASHRC

"$CONDA_EXE_PATH" init bash
if [ $? -ne 0 ]; then
    echo "[ERROR] conda init bash failed."
    echo "Restoring ~/.bashrc backup."
    if [ -f "$BACKUP_ROOT/.bashrc.backup" ]; then
        cp -a "$BACKUP_ROOT/.bashrc.backup" "$HOME/.bashrc"
    fi
    exit 1
fi
echo

# ------------------------------------------------------------
# 5. Add PATH de-duplication / conda precedence block
# ------------------------------------------------------------
echo "===== Adding PATH normalization ====="

python3 - "$HOME/.bashrc" <<'PY_PATH'
import sys
from pathlib import Path

path = Path(sys.argv[1])
lines = path.read_text(errors="replace").splitlines(keepends=True)

begin = "# >>> conda path repair >>>"
end = "# <<< conda path repair <<<"

out = []
inside = False

for line in lines:
    s = line.strip()

    if s == begin:
        inside = True
        continue

    if inside:
        if s == end:
            inside = False
        continue

    if s == end:
        continue

    out.append(line)

block = r'''
# >>> conda path repair >>>
# De-duplicate PATH while preserving order.
__repair_dedupe_path() {
    local __old_path="$PATH"
    local __new_path=""
    local __entry
    local __old_ifs="$IFS"

    IFS=':'
    for __entry in $__old_path; do
        [ -z "$__entry" ] && continue

        case ":$__new_path:" in
            *":$__entry:"*)
                ;;
            *)
                if [ -z "$__new_path" ]; then
                    __new_path="$__entry"
                else
                    __new_path="$__new_path:$__entry"
                fi
                ;;
        esac
    done

    IFS="$__old_ifs"
    PATH="$__new_path"
    export PATH
}

__repair_dedupe_path

# When conda is active, force the active environment's bin to the front.
if [ -n "${CONDA_PREFIX:-}" ] && [ -d "$CONDA_PREFIX/bin" ]; then
    __new_path="$CONDA_PREFIX/bin"
    __old_ifs="$IFS"
    IFS=':'

    for __entry in $PATH; do
        [ -z "$__entry" ] && continue
        [ "$__entry" = "$CONDA_PREFIX/bin" ] && continue

        case ":$__new_path:" in
            *":$__entry:"*)
                ;;
            *)
                __new_path="$__new_path:$__entry"
                ;;
        esac
    done

    IFS="$__old_ifs"
    PATH="$__new_path"
    export PATH

    unset __new_path __old_ifs __entry
fi

unset -f __repair_dedupe_path
# <<< conda path repair <<<
'''

path.write_text("".join(out).rstrip() + "\n\n" + block.strip() + "\n")
print("[OK] PATH repair block installed.")
PY_PATH

echo

# ------------------------------------------------------------
# 6. Reinstall pip into conda base
# ------------------------------------------------------------
echo "===== Reinstalling pip in conda base ====="

"$CONDA_EXE_PATH" install -n base --force-reinstall pip -y

if [ $? -eq 0 ]; then
    echo "[OK] pip reinstalled in conda base."
else
    echo "[WARN] conda pip reinstall failed."
    echo "       PATH/bashrc repair was still applied."
fi
echo

# ------------------------------------------------------------
# 7. Verification helper
# ------------------------------------------------------------
VERIFY="$HOME/check_conda_pip_after_repair.sh"

cat > "$VERIFY" <<'VERIFY_EOF'
#!/usr/bin/env bash

echo "=========================================="
echo " Conda / Pip Verification"
echo "=========================================="
echo

echo "CONDA_PREFIX      : ${CONDA_PREFIX:-<none>}"
echo "CONDA_DEFAULT_ENV : ${CONDA_DEFAULT_ENV:-<none>}"
echo

echo "===== python ====="
command -v python 2>/dev/null || true
python --version 2>/dev/null || true
echo

echo "===== pip ====="
command -v pip 2>/dev/null || true
pip -V 2>/dev/null || true
echo

echo "===== python -m pip ====="
python -m pip -V 2>/dev/null || true
echo

echo "===== type -a pip ====="
type -a pip 2>/dev/null || true
echo

echo "===== PATH first 20 ====="
echo "$PATH" | tr ':' '\n' | nl -ba | head -20
echo

if [ -n "${CONDA_PREFIX:-}" ]; then
    py="$(command -v python 2>/dev/null)"
    pp="$(command -v pip 2>/dev/null)"

    echo "===== checks ====="

    case "$py" in
        "$CONDA_PREFIX"/bin/*)
            echo "[OK]   python belongs to active conda env"
            ;;
        *)
            echo "[FAIL] python does NOT belong to active conda env"
            ;;
    esac

    case "$pp" in
        "$CONDA_PREFIX"/bin/*)
            echo "[OK]   pip belongs to active conda env"
            ;;
        *)
            echo "[FAIL] pip does NOT belong to active conda env"
            ;;
    esac

    if python -m pip -V 2>/dev/null | grep -F "$CONDA_PREFIX/" >/dev/null 2>&1; then
        echo "[OK]   python -m pip loads from active conda env"
    else
        echo "[FAIL] python -m pip loads from outside active conda env"
    fi
fi
VERIFY_EOF

chmod +x "$VERIFY"

echo "============================================================"
echo " Repair finished"
echo "============================================================"
echo
echo "Your original files were backed up to:"
echo "  $BACKUP_ROOT"
echo
echo "IMPORTANT: restart the current Bash process:"
echo
echo "  exec bash"
echo
echo "Then run:"
echo
echo "  conda activate base"
echo "  $VERIFY"
echo
echo "Expected pip executable:"
echo "  $CONDA_BASE/bin/pip"
echo
echo "To restore the old ~/.bashrc if needed:"
echo "  cp '$BACKUP_ROOT/.bashrc.backup' ~/.bashrc"
echo
