#include <cerrno>
#include <cstddef>
#include <cstdint>

extern "C" {
extern char _end;

void *__dso_handle = nullptr;

/* HTS_Hardware_Init.cpp weak 참조 대체: 스택 성장 하한(낮은 RAM 주소) */
uint32_t __stack_bottom__ = 0x20020000u - 0x8000u;

void *_sbrk(ptrdiff_t incr)
{
    static unsigned char *heap_end;
    if (heap_end == nullptr) {
        heap_end = reinterpret_cast<unsigned char *>(&_end);
    }
    unsigned char *const prev = heap_end;
    auto *const heap_limit =
        reinterpret_cast<unsigned char *>(&__stack_bottom__);
    if (heap_end + incr > heap_limit) {
        errno = ENOMEM;
        return reinterpret_cast<void *>(-1);
    }
    heap_end += incr;
    return prev;
}

int _kill(int, int) { return -1; }
int _getpid(void) { return 1; }
void _exit(int) { for (;;) { __asm volatile("wfi"); } }
}
