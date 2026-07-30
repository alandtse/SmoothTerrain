#include "HookUtil.hpp"

#include "PCH.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

using namespace SmoothTerrain;

auto HookUtil::findDirectBranches(std::uintptr_t fnStart,
                                  std::size_t window,
                                  std::uintptr_t target) -> std::vector<DirectBranch>
{
    constexpr std::uint8_t CALL_REL32_OPCODE = 0xE8;
    constexpr std::uint8_t JMP_REL32_OPCODE = 0xE9;
    constexpr std::size_t BRANCH_SIZE = 1 + sizeof(std::int32_t); /**< opcode byte plus rel32 displacement */

    // Viewing the scan window as a span keeps the byte reads below free of raw pointer arithmetic
    const std::span<const std::uint8_t> code {reinterpret_cast<const std::uint8_t*>(fnStart), window};

    std::vector<DirectBranch> branches;
    for (std::size_t i = 0; i + BRANCH_SIZE <= window; ++i) {
        const std::uint8_t opcode = code.subspan(i).front();
        if (opcode != CALL_REL32_OPCODE && opcode != JMP_REL32_OPCODE) {
            continue;
        }
        std::int32_t rel = 0;
        std::memcpy(&rel, code.subspan(i + 1, sizeof(rel)).data(), sizeof(rel));
        if (fnStart + i + BRANCH_SIZE + rel == target) {
            branches.push_back({.address = fnStart + i, .isJump = opcode == JMP_REL32_OPCODE});
        }
    }
    return branches;
}
