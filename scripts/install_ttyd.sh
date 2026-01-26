#!/usr/bin/env bash

# Install ttyd (web terminal) for bascorro_studio
# This script can be run standalone or called from the main script

set -euo pipefail

TTYD_VERSION="${TTYD_VERSION:-1.7.7}"
NON_INTERACTIVE="${NON_INTERACTIVE:-0}"

echo "Installing ttyd (web terminal server) v${TTYD_VERSION}..."

# Check if already installed
if command -v ttyd >/dev/null 2>&1; then
  current_version=$(ttyd --version 2>&1 | head -n1 || echo "unknown")
  echo "ttyd is already installed: $current_version"

  # Skip prompt if non-interactive mode
  if [[ "$NON_INTERACTIVE" == "1" ]]; then
    echo "Non-interactive mode: skipping reinstall"
    exit 0
  fi

  read -r -p "Reinstall? [y/N] " reply
  case "$reply" in
    y|Y|yes|YES) ;;
    *) echo "Skipping installation."; exit 0 ;;
  esac
fi

# Detect architecture
arch="$(dpkg --print-architecture 2>/dev/null || uname -m)"
case "$arch" in
  amd64|x86_64) ttyd_arch="x86_64" ;;
  arm64|aarch64) ttyd_arch="aarch64" ;;
  armhf|armv7l) ttyd_arch="armhf" ;;
  i386|i686) ttyd_arch="i686" ;;
  *)
    echo "ERROR: Unsupported architecture: $arch" >&2
    echo "Please install ttyd manually from https://github.com/tsl0922/ttyd/releases" >&2
    exit 1
    ;;
esac

# Download ttyd binary
download_url="https://github.com/tsl0922/ttyd/releases/download/${TTYD_VERSION}/ttyd.${ttyd_arch}"
echo "Downloading ttyd from: $download_url"

temp_file="/tmp/ttyd-$$"
if ! curl -fsSL -o "$temp_file" "$download_url"; then
  echo "ERROR: Failed to download ttyd" >&2
  rm -f "$temp_file"
  exit 1
fi

# Install to /usr/local/bin (requires sudo) or ~/.local/bin (user-local)
if [[ $EUID -eq 0 ]]; then
  # Running as root
  install_dir="/usr/local/bin"
  install -m 0755 "$temp_file" "$install_dir/ttyd"
  echo "Installed ttyd to $install_dir/ttyd"
else
  # Try sudo first, fallback to user install
  if sudo -n true 2>/dev/null; then
    install_dir="/usr/local/bin"
    sudo install -m 0755 "$temp_file" "$install_dir/ttyd"
    echo "Installed ttyd to $install_dir/ttyd (via sudo)"
  else
    echo "No sudo access. Installing to user directory..."
    install_dir="$HOME/.local/bin"
    mkdir -p "$install_dir"
    install -m 0755 "$temp_file" "$install_dir/ttyd"
    echo "Installed ttyd to $install_dir/ttyd"

    # Check if in PATH
    if [[ ":$PATH:" != *":$install_dir:"* ]]; then
      echo ""
      echo "WARNING: $install_dir is not in your PATH"
      echo "Add this to your ~/.bashrc or ~/.zshrc:"
      echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
    fi
  fi
fi

rm -f "$temp_file"

# Verify installation
if command -v ttyd >/dev/null 2>&1; then
  echo ""
  echo "✓ ttyd installed successfully!"
  ttyd --version
  echo ""
  echo "You can now run bascorro_studio with web terminal support."
else
  echo ""
  echo "WARNING: ttyd was installed but not found in PATH" >&2
  echo "You may need to restart your shell or update your PATH." >&2
  exit 1
fi
