#pragma once
/// @file  HTS_CXX17_Atomic_Safe.h
/// @brief MSVC: Windows Kit / SAL이 `_Order`·`_Success`·`_Failure` 등을 정의한 뒤
///        `<atomic>`이 파싱되면 std::atomic 내부가 깨질 수 있음. 본 래퍼로 머신러만 조정(알고리즘 무관).
#if defined(_MSC_VER)
#pragma push_macro("_Order")
#pragma push_macro("_Success")
#pragma push_macro("_Failure")
#pragma push_macro("_Combine_cas_memory_orders")
#if defined(_Order)
#undef _Order
#endif
#if defined(_Success)
#undef _Success
#endif
#if defined(_Failure)
#undef _Failure
#endif
#if defined(_Combine_cas_memory_orders)
#undef _Combine_cas_memory_orders
#endif
#endif
#include <atomic>
#if defined(_MSC_VER)
#pragma pop_macro("_Combine_cas_memory_orders")
#pragma pop_macro("_Failure")
#pragma pop_macro("_Success")
#pragma pop_macro("_Order")
#endif
