// Exercises the pieces that break when a container's ABI does not match the
// target: the dynamic loader, libstdc++, exceptions across a shared boundary,
// and the std::string / std::vector layouts that changed at the C++11 ABI
// switch. A pure-libc "hello world" would link and run against almost
// anything and prove nothing.
#include <cstdio>
#include <gnu/libc-version.h>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

int main() {
    std::printf("  glibc     %s\n", gnu_get_libc_version());
    std::printf("  __GLIBCXX__ %ld\n", (long)__GLIBCXX__);
    std::printf("  _GLIBCXX_USE_CXX11_ABI %d\n", _GLIBCXX_USE_CXX11_ABI);

    std::vector<int> v(1000);
    std::iota(v.begin(), v.end(), 1);
    std::printf("  vector sum %ld\n", (long)std::accumulate(v.begin(), v.end(), 0L));

    auto s = std::make_shared<std::string>("string and shared_ptr survive");
    std::printf("  %s (len %zu)\n", s->c_str(), s->size());

    try {
        throw std::runtime_error("exception unwinding works");
    } catch (const std::exception& e) {
        std::printf("  %s\n", e.what());
    }
    return 0;
}
