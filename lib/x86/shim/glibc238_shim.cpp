// glibc238_shim.cpp — lib/x86 预编译库（ffmpeg 7.x / libctb）符号垫片
//
// 这些库在较新的系统（glibc >= 2.38 / GCC >= 13）上构建，引用了下列版本符号：
//   __isoc23_sscanf / __isoc23_strtol      @ GLIBC_2.38   (libavcodec)
//   fmod / fmodf                           @ GLIBC_2.38   (libavformat/libavutil/libavfilter)
//   std::ios_base_library_init()           @ GLIBCXX_3.4.32 (libctb)
// 而官方构建基线是 ubuntu:22.04（见 Dockerfile.base，glibc 2.35 / libstdc++ 3.4.30），
// 在其上链接 ziyan_gcs 会报 undefined reference，运行时亦无法通过版本检查。
//
// 本垫片以「同名版本节点」重新导出等价实现：
//   - isoc23 系列转发到 sscanf/strtol（语义差异仅为 %i/%d 对 0b/0o 前缀的解析，不影响 ffmpeg）
//   - fmod/fmodf 经 .symver 绑定到 libm 的 GLIBC_2.2.5 旧版符号（避免转发调用回落到
//     本垫片自己导出的 fmod@GLIBC_2.38 造成自递归）
//   - ios_base_library_init 以静态 ios_base::Init 对象完成等价的 iostream 初始化
//     （GCC13 的该符号即做此事；静态 Init 在新老 libstdc++ 下均正确且幂等）
//
// 仅当构建环境缺少这些符号时由 CMake 启用并链接进 ziyan_gcs（见 CMakeLists.txt
// 主程序段），新系统（glibc >= 2.38）自动跳过，不会遮蔽真实实现。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <iostream>

// 静态 Init：本垫片被加载即完成 iostream 初始化，等价 GCC13 的 ios_base_library_init
namespace
{
    std::ios_base::Init g_ios_init_once;
}

// std::ios_base_library_init()（直接以 mangled 名导出，见 glibc238_shim.map）
extern "C" void _ZSt21ios_base_library_initv()
{
    // 初始化已由上方静态对象完成，这里保持与真实实现相同的空函数体语义
}

extern "C" int __isoc23_sscanf(const char *str, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int result = vsscanf(str, format, ap);
    va_end(ap);
    return result;
}

extern "C" long __isoc23_strtol(const char *nptr, char **endptr, int base)
{
    return strtol(nptr, endptr, base);
}

// 经 .symver 把内部调用绑定到 libm 的旧版符号（GLIBC_2.2.5），杜绝自递归
extern "C" {
double fmod_glibc225(double x, double y);
float fmodf_glibc225(float x, float y);
}
__asm__(".symver fmod_glibc225, fmod@GLIBC_2.2.5");
__asm__(".symver fmodf_glibc225, fmodf@GLIBC_2.2.5");

extern "C" double fmod(double x, double y) { return fmod_glibc225(x, y); }
extern "C" float fmodf(float x, float y) { return fmodf_glibc225(x, y); }
