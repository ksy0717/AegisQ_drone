#!/bin/bash
# Install only the WHIP-capable Rust plugin while keeping Ubuntu's integrated
# GStreamer/libcamera stack. GStreamer 1.29.x is development-only and must not
# replace the vehicle runtime as a side effect of enabling WHIP.
set -euo pipefail

GST_RS_TAG="${GST_RS_TAG:-0.12.11}"
GST_RS_COMMIT="${GST_RS_COMMIT:-177dd1af8ddcd9f6755ea7007b63169c6f2a3ee7}"
GST_PLUGIN_DIR="${GST_PLUGIN_DIR:-/usr/local/lib/gstreamer-1.0}"
JOBS="${JOBS:-2}"

[ "$(id -u)" -eq 0 ] || {
  echo "run with sudo" >&2
  exit 1
}

gst_version="$(pkg-config --modversion gstreamer-1.0)"
case "$gst_version" in
  1.24.*|1.26.*|1.28.*)
    ;;
  *)
    echo "unsupported vehicle GStreamer version: $gst_version" >&2
    echo "validated stable series are 1.24, 1.26, and 1.28" >&2
    exit 1
    ;;
esac

if [ "${FORCE_GST_WHIP_REBUILD:-0}" != "1" ]; then
  installed_version="$(
    GST_PLUGIN_PATH="$GST_PLUGIN_DIR" gst-inspect-1.0 whipclientsink 2>/dev/null \
      | awk '$1 == "Version" {print $2; exit}' || true
  )"
  case "$installed_version" in
    "$GST_RS_TAG"|"$GST_RS_TAG"-*)
      echo "   whipclientsink $installed_version is already installed"
      exit 0
      ;;
  esac
fi

apt-get install -y --no-install-recommends \
  cargo git build-essential pkg-config ca-certificates \
  libssl-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev gstreamer1.0-nice

rust_version="$(rustc --version | awk '{print $2}')"
if ! printf '%s\n%s\n' "1.70.0" "$rust_version" | sort -V -C; then
  echo "rustc >= 1.70 is required; found $rust_version" >&2
  exit 1
fi

work_dir="$(mktemp -d)"
case "$work_dir" in
  /tmp/*)
    ;;
  *)
    echo "refusing unexpected temporary directory: $work_dir" >&2
    exit 1
    ;;
esac
cleanup() {
  rm -rf -- "$work_dir"
}
trap cleanup EXIT

git clone --depth 1 --branch "$GST_RS_TAG" \
  https://github.com/GStreamer/gst-plugins-rs.git "$work_dir/gst-plugins-rs"
actual_commit="$(git -C "$work_dir/gst-plugins-rs" rev-parse HEAD)"
if [ "$actual_commit" != "$GST_RS_COMMIT" ]; then
  echo "gst-plugins-rs tag resolved to unexpected commit: $actual_commit" >&2
  exit 1
fi

# cargo-c in Ubuntu 24.04 is older than this release's metadata requirement.
# A cdylib build produces the same loadable GStreamer plugin without cargo-c.
cargo build \
  --manifest-path "$work_dir/gst-plugins-rs/Cargo.toml" \
  --release --locked -p gst-plugin-webrtc -j "$JOBS"

plugin="$work_dir/gst-plugins-rs/target/release/libgstrswebrtc.so"
[ -s "$plugin" ] || {
  echo "WHIP plugin build did not produce $plugin" >&2
  exit 1
}

install -d -m755 "$GST_PLUGIN_DIR"
install -m755 "$plugin" "$GST_PLUGIN_DIR/libgstrswebrtc.so"

GST_PLUGIN_PATH="$GST_PLUGIN_DIR" gst-inspect-1.0 whipclientsink >/dev/null
echo "   installed gst-plugins-rs $GST_RS_TAG into $GST_PLUGIN_DIR"
