#pragma once
// 단일 정의(ODR): PLUTO_검증·시뮬 툴이 HTS_LIM보다 먼저 include path에 있을 때
// 동일 복제본이 중복 포함되어 constexpr 본문이 2벌(C2084)로 링크되는 것을 막는다.
#include "../../HTS_LIM/HTS_BitOps.h"
