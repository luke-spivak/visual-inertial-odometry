#!/usr/bin/env bash
# buildenv.sh -- arm64 Debian trixie container matching viopi's ABI.
#
#   ./buildenv.sh build           build the image
#   ./buildenv.sh verify          prove a binary built here runs on the Pi
#   ./buildenv.sh shell           interactive shell, cwd mounted at /work
#   ./buildenv.sh run <cmd...>    run one command in the container
#
# Not cross-compilation on Apple Silicon: Mac and Pi are both arm64, so this
# runs natively. It is a faster arm64 machine, not an emulator.
#
# `verify` is the point of this script. "The image built" proves nothing --
# a bookworm container builds just as happily and yields binaries that only
# fail once they reach the Pi. The gate is a binary compiled here, executed
# there.
set -euo pipefail

IMAGE="${IMAGE:-vio-build:trixie}"
PI="${PI:-viopi}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

need_docker() {
    docker info >/dev/null 2>&1 || {
        echo "Docker daemon is not reachable. Start Docker Desktop and retry." >&2
        exit 1
    }
}

drun() {
    docker run --rm --platform linux/arm64 -v "$PWD":/work -w /work "$IMAGE" "$@"
}

case "${1:-}" in
build)
    need_docker
    docker build --platform linux/arm64 -t "$IMAGE" "$HERE"
    ;;

shell)
    need_docker
    docker run --rm -it --platform linux/arm64 -v "$PWD":/work -w /work "$IMAGE" bash
    ;;

run)
    need_docker
    shift
    drun "$@"
    ;;

verify)
    need_docker
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' EXIT
    cp "$HERE/abi_probe.cpp" "$tmp/"

    echo "=== 1. container identity ==="
    arch=$(docker run --rm --platform linux/arm64 "$IMAGE" uname -m)
    cglibc=$(docker run --rm --platform linux/arm64 "$IMAGE" \
             sh -c 'ldd --version | head -1' | grep -o '[0-9]\+\.[0-9]\+$')
    cgcc=$(docker run --rm --platform linux/arm64 "$IMAGE" gcc -dumpfullversion)
    printf "  arch %s   glibc %s   gcc %s\n" "$arch" "$cglibc" "$cgcc"
    [ "$arch" = "aarch64" ] || { echo "  FAIL: container is $arch, Pi is aarch64"; exit 1; }

    echo "=== 2. the Pi it has to match ==="
    pglibc=$(ssh "$PI" 'ldd --version | head -1' | grep -o '[0-9]\+\.[0-9]\+$')
    pgcc=$(ssh "$PI" 'gcc -dumpfullversion')
    printf "  arch aarch64   glibc %s   gcc %s\n" "$pglibc" "$pgcc"
    if [ "$cglibc" != "$pglibc" ]; then
        echo "  FAIL: glibc $cglibc vs $pglibc."
        echo "  This is the failure the base image tag exists to prevent, and it"
        echo "  does not surface as a build error -- only as binaries that look"
        echo "  corrupt on the Pi. Fix the FROM line before building anything."
        exit 1
    fi
    echo "  glibc matches"

    echo "=== 3. compile in the container ==="
    ( cd "$tmp" && docker run --rm --platform linux/arm64 -v "$tmp":/work -w /work \
        "$IMAGE" g++ -O2 -o abi_probe abi_probe.cpp )
    file "$tmp/abi_probe" | sed 's/^/  /'

    echo "=== 4. run it in the container ==="
    here_out=$(docker run --rm --platform linux/arm64 -v "$tmp":/work -w /work \
               "$IMAGE" ./abi_probe)
    echo "$here_out"

    echo "=== 5. run the same binary on the Pi ==="
    scp -q "$tmp/abi_probe" "$PI:/tmp/abi_probe"
    there_out=$(ssh "$PI" 'chmod +x /tmp/abi_probe && /tmp/abi_probe; echo "exit=$?"')
    ssh "$PI" 'rm -f /tmp/abi_probe'
    echo "$there_out"

    echo "=== 6. runtime libraries the Pi must supply ==="
    # An ABI-clean binary still will not start if the shared objects are absent
    # or a different version. Both sides are the same Debian release, so apt on
    # the Pi yields byte-identical versions -- but that is a claim worth
    # checking, not assuming, and it is the other half of why the base image
    # tag has to match.
    mismatch=0
    for pkg in libopencv-core410 libopencv-imgproc410 libopencv-calib3d410 \
               libceres4t64 libgoogle-glog0v6t64 libgflags2.2; do
        cver=$(docker run --rm --platform linux/arm64 "$IMAGE" \
               dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || echo "-")
        pver=$(ssh "$PI" "apt-cache policy $pkg 2>/dev/null | awk '/Candidate:/{print \$2}'")
        pinst=$(ssh "$PI" "dpkg-query -W -f='\${Version}' $pkg 2>/dev/null || true")
        state="available"
        [ -n "$pinst" ] && state="installed"
        if [ "$cver" != "-" ] && [ "$cver" != "$pver" ]; then
            state="MISMATCH"; mismatch=1
        fi
        printf "  %-24s container=%-24s pi=%-24s %s\n" \
               "$pkg" "$cver" "${pver:-NONE}" "$state"
    done
    [ "$mismatch" = 0 ] || { echo "  FAIL: version skew, a build here will not run there"; exit 1; }

    echo "=== verdict ==="
    if [ "$here_out" = "${there_out%$'\n'exit=0}" ]; then
        echo "  PASS -- identical output in the container and on the Pi."
    else
        echo "  FAIL -- output differs between container and Pi."
        exit 1
    fi
    ;;

*)
    sed -n '2,14p' "$0" | sed 's/^# \?//'
    exit 1
    ;;
esac
