#pragma once

#include "PCH.h"

#include <cstdint>

namespace SmoothTerrain {

/**
 * @brief Keeps the engine's decal projection working on subdivided landscape
 *
 * Decals that clip themselves to the surface they land on (blood, scorch marks, and the
 * footprints mods build on the same system) are BSTempEffectSimpleDecal: on creation the
 * engine walks the triangles of the geometry under the decal and clips them against the
 * decal's box. Those triangles normally come from the shape's own CPU side index data, which
 * landscape shapes do not have - every land quad shares one static 17x17 index list instead,
 * so the collector special cases the landscape material and fetches that engine global.
 *
 * The loop bound, however, still comes from the shape's own triangleCount. On a subdivided
 * quad that is 4^level times the 512 triangles the shared list describes, so the collector
 * runs off the end of it: the decal is projected onto whatever memory follows (misplaced,
 * stretched or invisible decals) and the vertex fetch behind those junk indices eventually
 * leaves the vertex buffer and crashes the game.
 *
 * This class hands the collector the matching index list instead. One call-site hook on the
 * collector records the subdivided list belonging to the shape being decaled, a second one on
 * the engine's land-index-list getter returns it in place of the 17x17 list. Both are
 * call-site patches, so other plugins can still detour either function, and anything that is
 * not one of our subdivided quads keeps using the engine's own list.
 */
class DecalFix {
public:
    DecalFix() = delete;

    /**
     * @brief Installs the decal hooks; requires SKSE::AllocTrampoline beforehand
     *
     * Does nothing when subdivision is disabled or the runtime is unsupported - the vanilla
     * land mesh matches the engine's index list and needs no fix.
     */
    static void install();

private:
    /**
     * @brief Call-site hook around the decal builder's triangle collection pass
     */
    struct CollectHook {
        static auto thunk(RE::BSTempEffectSimpleDecal* decal,
                          RE::BSTriShape* shape) -> std::uintptr_t;
        static inline REL::Relocation<decltype(thunk)> s_func; /**< The unpatched collector */

        /**
         * @brief Index list of the land shape currently being decaled on this thread
         *
         * Only set while the collector runs, and only for shapes this plugin subdivided;
         * nullptr means "not ours", which leaves the engine's own list in charge. Decals are
         * built on more than one thread, hence thread local.
         */
        static inline thread_local const std::uint16_t* s_indices = nullptr;
    };

    /**
     * @brief Call-site hook on the engine's shared land index list getter
     */
    struct IndexListHook {
        static auto thunk() -> const std::uint16_t*;
        static inline REL::Relocation<decltype(thunk)> s_func; /**< The unpatched getter */
    };

    /**
     * @brief Looks up the index list that describes a shape's triangles
     *
     * @param shape The shape a decal is being projected onto
     * @return const std::uint16_t* The subdivided list for that shape, or nullptr for
     *         anything this plugin did not build (including vanilla 17x17 land quads)
     */
    static auto landIndicesFor(RE::BSTriShape* shape) -> const std::uint16_t*;
};

} // namespace SmoothTerrain
