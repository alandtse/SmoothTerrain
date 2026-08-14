#pragma once

#include "TerrainSubdivision.hpp"

#include "PCH.h"

#include "RE/C/CellMopp.h"
#include "RE/H/HeightFieldCInfo.h"
#include "RE/L/LandCollisionDesc.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace SmoothTerrain {

/**
 * @brief Gives the landscape collision the same smoothed surface the subdivided mesh renders
 *
 * The engine collides against land through a sampled height field, not through the render mesh:
 * for each cell quadrant it copies LoadedLandData::heights[quad] - a 17x17 grid at 128 unit
 * spacing - into a quantized hkpCompressedSampledHeightFieldShape and gives that to a static
 * rigid body. Subdividing the mesh therefore changes nothing about what the player walks on:
 * every original LAND vertex keeps its exact height, so the two surfaces still meet at the grid
 * points, but in between the smoothed mesh curves while collision stays on the vanilla flat
 * triangles. That is the gap this class closes.
 *
 * It closes it at the source rather than by rewriting shapes afterwards. The height field's
 * resolution, spacing and heights all come from one construction info block that the collision
 * builder fills on its stack, so a call site hook on the shape's init reads that block, swaps in
 * a finer grid sampled from the same Catmull-Rom surface TerrainSubdivision draws, and restores
 * it when the call returns. Everything downstream is the engine's own code path working on
 * honest inputs: it quantizes our samples into its compressed shape (a copy - no pointer of ours
 * survives the call), derives the shape's bounds from the range we hand it, and registers the
 * body with the havok world exactly as it always did. Nothing havok-side is created, mutated or
 * released by this plugin, and no state has to be tracked or unwound afterwards.
 *
 * The level is always iSubdivisions, so the surface underfoot is the one on screen. Two
 * differences from the render path are deliberate:
 *  - No distance falloff. Collision is refined for every land quad that loads, at the full
 *    level, because anything in the loaded grid can stand on it while the mesh a mile away is
 *    only ever looked at. It also makes the collision surface continuous everywhere by
 *    construction: neighboring quads always sample the same spline at the same level, so no
 *    edge stitching is needed (see TerrainSubdivision::buildQuadHeightField).
 *  - Built during the cell load, unlike every mesh this plugin makes. Collision has to exist the
 *    moment the cell attaches, and the work is a fraction of what the surrounding engine code
 *    does anyway (sampling one float grid, against building four meshes and uploading them).
 *
 * A quad rendering below full level (the falloff's gradient, or vanilla past it) therefore
 * collides against the finer surface regardless. The two differ by at most the spline's
 * deviation from the coarser mesh's chords, which is bounded by fMaxRise upward, and only ever
 * out where the falloff has already decided the player is not - the full-level square is
 * centered on the player's own quad, so what they stand on always matches what they see.
 */
class TerrainCollision {
public:
    /**
     * @brief Installs the call site hooks; requires SKSE::AllocTrampoline beforehand
     */
    static void install();

    TerrainCollision() = delete;

private:
    /**
     * @brief The subdivided height grids of one cell, alive for the duration of one collision build
     *
     * Lives on the outer hook's stack and is published to the inner hook through s_refinement.
     * The engine copies every sample it is given before the inner call returns, so nothing here
     * has to outlive the build.
     */
    struct Refinement {
        const float* base {}; /**< The descriptor's height table, the origin quadrants are keyed off */
        std::uint32_t steps {}; /**< Fine grid steps per vanilla grid step (1 << level) */
        std::array<TerrainSubdivision::QuadHeightField, TerrainSubdivision::K_QUADS_PER_CELL> quads;

        /**
         * @brief The refined grid standing in for the one a cinfo points at, if there is one
         *
         * Identifies the quadrant by where its height array sits in the cell's table, which is
         * exact integer arithmetic rather than a guess at call order, and refuses anything that
         * is not one of the four vanilla quadrant grids of this very cell.
         *
         * @param cinfo The construction info the engine is about to build a shape from
         * @return const TerrainSubdivision::QuadHeightField* The replacement, or nullptr to leave
         *         the build alone
         */
        [[nodiscard]] auto fieldFor(const RE::HeightFieldCInfo* cinfo) const
            -> const TerrainSubdivision::QuadHeightField*;
    };

    /**
     * @brief Call site hook around the engine's land collision builder
     *
     * Samples the cell's four refined grids up front - one whole-cell height grid serves all
     * four quadrants - and publishes them for the nested shape builds below.
     */
    struct BuildHook {
        static auto thunk(RE::CellMopp* cellMopp,
                          const RE::LandCollisionDesc* desc) -> bool;
        static inline REL::Relocation<decltype(thunk)> s_func; /**< The unpatched builder */
    };

    /**
     * @brief Call site hook around the engine's height field shape init
     *
     * Runs inside BuildHook, on the same thread, once per quadrant.
     */
    struct InitHook {
        static auto thunk(void* shape,
                          RE::HeightFieldCInfo* cinfo) -> std::uintptr_t;
        static inline REL::Relocation<decltype(thunk)> s_func; /**< The unpatched init */
    };

    /**
     * @brief Samples a cell's four refined height grids from its descriptor
     *
     * @param desc The descriptor the engine is about to build collision from
     * @param refinement Filled in on success
     * @return bool False when this is not the vanilla landscape descriptor (another plugin's
     *         layout, or a shape count / grid dimension this code was not written against), in
     *         which case the build stays vanilla
     */
    [[nodiscard]] static auto buildRefinement(const RE::LandCollisionDesc* desc,
                                              Refinement& refinement) -> bool;

    /**
     * @brief The cell whose collision is being built on this thread, or nullptr
     *
     * Thread local because the engine loads cells on several threads at once; the inner hook
     * always runs inside its own thread's outer call, and the saved-and-restored handoff keeps
     * the pairing correct even if the engine ever nests two builds.
     */
    static thread_local inline const Refinement* s_refinement = nullptr;
};

} // namespace SmoothTerrain
