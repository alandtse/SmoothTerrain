#pragma once

#include "PCH.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SmoothTerrain::HookUtil {

/**
 * @brief A direct (rel32) call or jump instruction inside a function body
 */
struct DirectBranch {
    std::uintptr_t address {}; /**< Address of the opcode byte */
    bool isJump {}; /**< True for E9 (tail call), false for E8 (call) */
};

/**
 * @brief Finds every direct rel32 call / jump to a target inside a function body
 *
 * Locating branches by their resolved target instead of by a byte signature keeps the scan
 * working across game versions: the call site moves, but it still points at the same
 * (Address Library resolved) function.
 *
 * @param fnStart Start of the function to scan
 * @param window Bytes to scan; overshooting into the next function is harmless as long as
 *        every branch to the target deserves the same treatment
 * @param target Branch target that must match
 * @return std::vector<DirectBranch> Every matching instruction, in address order
 */
[[nodiscard]] auto findDirectBranches(std::uintptr_t fnStart,
                                      std::size_t window,
                                      std::uintptr_t target) -> std::vector<DirectBranch>;

/**
 * @brief Redirects every direct call / jump to a target inside a function to a replacement
 *
 * Patching the call sites rather than detouring the target's entry leaves the target itself
 * untouched, so other plugins (Community Shaders, ENB, ...) can still hook it and a
 * replacement that calls the original address runs through whatever they installed.
 *
 * @param fnStart Start of the function to scan
 * @param window Bytes to scan
 * @param target Branch target to redirect away from
 * @param replacement Function to branch to instead
 * @return std::vector<DirectBranch> The patched sites (empty when the target was not found)
 */
template <class F>
auto redirectBranches(std::uintptr_t fnStart,
                      std::size_t window,
                      std::uintptr_t target,
                      F replacement) -> std::vector<DirectBranch>
{
    constexpr std::size_t BRANCH_SIZE = 5; /**< Opcode byte plus rel32 displacement */

    const auto sites = findDirectBranches(fnStart, window, target);
    auto& trampoline = SKSE::GetTrampoline();
    for (const auto& site : sites) {
        // A tail jump has to stay a jump: the replacement returns straight to the caller of
        // the function we are patching
        if (site.isJump) {
            trampoline.write_branch<BRANCH_SIZE>(site.address, replacement);
        } else {
            trampoline.write_call<BRANCH_SIZE>(site.address, replacement);
        }
    }
    return sites;
}

} // namespace SmoothTerrain::HookUtil
