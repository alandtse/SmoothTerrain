#pragma once

#include "ConfigLoader.hpp"

#include "PCH.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

namespace SmoothTerrain {

/**
 * @brief Subdivides the engine-built landscape meshes for smoother terrain
 *
 * The engine builds one BSTriShape per cell quadrant from the LAND record's 17x17 height grid
 * (BuildQuadTriShape, see Offsets.hpp). This class hooks every call site of that builder,
 * lets the vanilla builder run, then replaces the shape's GPU buffers with a subdivided version:
 * every original vertex is kept bit-exact in place and new vertices are interpolated in between.
 * Heights follow a Catmull-Rom spline over the whole cell - the original verts act as fixed
 * knots and both sides of every knot share one tangent, so the faceted creases of the vanilla
 * mesh disappear; every other attribute is bilinear. Materials, collision, multibounds and the
 * scene graph are untouched, so the change is invisible to other plugins - only the mesh
 * density changes.
 *
 * Call sites are patched with the SKSE trampoline (write_call) instead of a function-entry
 * detour, so other mods (Community Shaders, ENB, ...) can still detour the builder or anything
 * around it; our call into the "original" goes through whatever they install.
 */
class TerrainSubdivision {
public:
    /**
     * @brief Installs the call-site hooks; requires SKSE::AllocTrampoline beforehand
     */
    static void install();

    /**
     * @brief Returns the index list of the subdivided quad with these counts, if there is one
     *
     * Everything about a quad's triangulation follows from its grid dimension, so the vertex
     * and triangle counts identify the shared index list of a subdivided quad exactly. Used by
     * DecalFix, which has to hand the engine's decal builder the triangles of the shape it is
     * projecting onto.
     *
     * @param vertexCount Vertex count of the shape in question
     * @param triangleCount Triangle count of the shape in question
     * @return std::span<const std::uint16_t> The list (3 indices per triangle), or an empty
     *         span when no subdivision level built at runtime matches those counts
     */
    [[nodiscard]] static auto findIndexData(std::uint32_t vertexCount,
                                            std::uint32_t triangleCount) -> std::span<const std::uint16_t>;

private:
    //
    // Vanilla land mesh constants (verified against SkyrimSE.exe 1.6.1170 and 1.5.97, which
    // share the landscape vertex layout and LoadedLandData field offsets byte for byte)
    //
    constexpr static std::uint32_t K_QUAD_COUNT = 4; /**< Quadrants per cell (SW, SE, NW, NE) */
    constexpr static std::uint32_t K_COARSE_DIM = 17; /**< Verts per quadrant side in the vanilla mesh */
    constexpr static std::uint32_t K_COARSE_VERTS = K_COARSE_DIM * K_COARSE_DIM; /**< 289 verts per vanilla quad */
    constexpr static std::uint32_t K_COARSE_TRIS
        = (K_COARSE_DIM - 1) * (K_COARSE_DIM - 1) * 2; /**< 512 triangles per vanilla quad */
    constexpr static std::uint32_t K_CELL_DIM
        = (2 * (K_COARSE_DIM - 1)) + 1; /**< 33 verts per cell side (4 quadrants share edges) */
    constexpr static float K_COARSE_STEP = 128.0F; /**< World units between two original land verts */
    constexpr static float K_QUAD_SIZE
        = static_cast<float>(K_COARSE_DIM - 1) * K_COARSE_STEP; /**< 2048 world units per quadrant side */
    constexpr static std::size_t K_VERTEX_STRIDE = 40; /**< Byte stride of the engine's landscape vertex layout */
    constexpr static std::size_t K_BLEND_LAYERS = 6; /**< Texture blend weights stored per land vertex */
    constexpr static std::size_t K_INDEX_DATA_SIZE = 16; /**< Size of the engine's index buffer wrapper */

    //
    // Interpolation and attribute encoding constants
    //
    constexpr static float K_HALF = 0.5F; /**< Midpoints, nearest-corner splits and the (v + 1) / 2 encoding */
    constexpr static float K_BYTE_MAX = 255.0F; /**< Upper clamp when rounding an interpolated byte attribute */
    constexpr static float K_UNIT_BYTE_SCALE = 127.5F; /**< The engine's (v + 1) * 127.5 byte encoding of [-1, 1] */

    //
    // IEEE 754 binary16 (half) and binary32 (float) field layouts, shared by halfToFloat and
    // floatToHalf; every mask and shift below is derived from these
    //
    constexpr static std::uint32_t K_HALF_MANTISSA_BITS = 10;
    constexpr static std::uint32_t K_HALF_EXPONENT_BITS = 5;
    constexpr static std::int32_t K_HALF_EXPONENT_BIAS = 15;
    constexpr static std::uint32_t K_FLOAT_MANTISSA_BITS = 23;
    constexpr static std::uint32_t K_FLOAT_EXPONENT_BITS = 8;
    constexpr static std::int32_t K_FLOAT_EXPONENT_BIAS = 127;

    constexpr static std::uint32_t K_HALF_SIGN_MASK = 1U << (K_HALF_EXPONENT_BITS + K_HALF_MANTISSA_BITS);
    constexpr static std::uint32_t K_HALF_EXPONENT_MASK = (1U << K_HALF_EXPONENT_BITS) - 1U;
    constexpr static std::uint32_t K_HALF_MANTISSA_MASK = (1U << K_HALF_MANTISSA_BITS) - 1U;
    constexpr static std::uint32_t K_HALF_IMPLICIT_ONE = 1U << K_HALF_MANTISSA_BITS;
    constexpr static std::uint32_t K_HALF_INFINITY = K_HALF_EXPONENT_MASK << K_HALF_MANTISSA_BITS;
    constexpr static std::uint32_t K_HALF_VALUE_MASK = K_HALF_SIGN_MASK - 1U; /**< everything but the sign bit */
    constexpr static std::uint32_t K_SIGN_SHIFT
        = (K_FLOAT_EXPONENT_BITS + K_FLOAT_MANTISSA_BITS) - (K_HALF_EXPONENT_BITS + K_HALF_MANTISSA_BITS);
    constexpr static std::uint32_t K_MANTISSA_SHIFT = K_FLOAT_MANTISSA_BITS - K_HALF_MANTISSA_BITS;
    constexpr static std::uint32_t K_FLOAT_EXPONENT_ALL_ONES = ((1U << K_FLOAT_EXPONENT_BITS) - 1U)
        << K_FLOAT_MANTISSA_BITS;
    constexpr static std::uint32_t K_FLOAT_MANTISSA_MASK = (1U << K_FLOAT_MANTISSA_BITS) - 1U;
    constexpr static std::uint32_t K_FLOAT_IMPLICIT_ONE = 1U << K_FLOAT_MANTISSA_BITS;
    constexpr static std::uint32_t K_FLOAT_ABS_MASK = K_FLOAT_EXPONENT_ALL_ONES | K_FLOAT_MANTISSA_MASK;
    constexpr static std::int32_t K_BIAS_DELTA = K_FLOAT_EXPONENT_BIAS - K_HALF_EXPONENT_BIAS;

    /**
     * @brief One vertex record of the engine's landscape vertex layout (stride 40,
     * vertex desc 0x000BB0080765040A)
     *
     * Encodings follow the vanilla builder: normals are biased bytes (signed char + 0x80),
     * tangent/bitangent components are (v + 1) * 127.5 bytes except tangent X which lives at
     * offset 12 as a (v + 1) * 0.5 float, and UVs are half floats.
     */
#pragma pack(push, 1)
    struct LandVertex {
        float posX {}; /**< 00: model-space X (quad-local, [-2048..0] or [0..2048]) */
        float posY {}; /**< 04: model-space Y */
        float posZ {}; /**< 08: model-space Z (the land height) */
        float tangentXEnc {}; /**< 12: (tangent.x + 1) * 0.5 */
        std::uint16_t u {}; /**< 16: half-float U */
        std::uint16_t v {}; /**< 18: half-float V */
        std::array<std::uint8_t, 3> normal {}; /**< 20: biased normal bytes */
        std::uint8_t tangentYEnc {}; /**< 23: (tangent.y + 1) * 127.5 */
        std::array<std::uint8_t, 3> bitangent {}; /**< 24: (bitangent + 1) * 127.5 bytes */
        std::uint8_t tangentZEnc {}; /**< 27: (tangent.z + 1) * 127.5 */
        std::array<std::uint8_t, 4> color {}; /**< 28: RGBA vertex color (A = 255) */
        std::array<std::uint8_t, K_BLEND_LAYERS> blend {}; /**< 32: texture blend weights, layer 0 = base */
        std::array<std::uint8_t, 2> pad {}; /**< 38: unused */
    };
#pragma pack(pop)
    static_assert(sizeof(LandVertex) == K_VERTEX_STRIDE);

    /**
     * @brief The engine's 16-byte shared index buffer wrapper (created by CreateIndexBuffer)
     */
    struct IndexBufferData {
        void* buffer; /**< 00: ID3D11Buffer* (R16_UINT) */
        volatile std::uint32_t refCount; /**< 08 */
        std::uint32_t pad0C; /**< 0C */
    };
    static_assert(sizeof(IndexBufferData) == K_INDEX_DATA_SIZE);

    /**
     * @brief Call-site hook around the engine's BuildQuadTriShape
     */
    struct BuildQuadHook {
        static auto thunk(RE::TESObjectLAND::LoadedLandData* data,
                          std::uint32_t quad) -> RE::BSTriShape*;
        static inline REL::Relocation<decltype(thunk)> s_func; /**< The unpatched builder entry */
    };

    static inline std::mutex s_indexBufferMutex; /**< Guards the index caches (land builds can run off-thread) */
    static inline std::array<IndexBufferData*, ConfigLoader::MAX_SUBDIVISIONS + 1>
        s_indexBuffers {}; /**< One engine-shared index buffer per subdivision level (slot 0 unused) */
    static inline std::array<std::vector<std::uint16_t>, ConfigLoader::MAX_SUBDIVISIONS + 1>
        s_indexData {}; /**< CPU copy of each level's index list; kept for the engine's decal builder,
                             which reads triangles from CPU memory (see DecalFix). Filled once per
                             level and never resized afterwards, so the data pointer stays valid. */

public:
    TerrainSubdivision() = delete;

private:
    /**
     * @brief Replaces a freshly built quad shape's buffers with a subdivided version
     *
     * On any unexpected input (foreign vertex layout, missing CPU copy, buffer creation
     * failure) the shape is left exactly as the vanilla builder made it.
     *
     * @param shape The vanilla-built 289-vert quad shape (modified in place)
     * @param data Loaded land data of the cell (heights and cell coordinates)
     * @param quad Quadrant index 0-3 (0 = SW, 1 = SE, 2 = NW, 3 = NE)
     * @param level Subdivision level 1-3
     * @return bool True when the shape now holds the subdivided buffers
     */
    static auto subdivide(RE::BSTriShape& shape,
                          const RE::TESObjectLAND::LoadedLandData& data,
                          std::uint32_t quad,
                          int level) -> bool;

    /**
     * @brief Assembles the full 33x33 cell height grid from the four 17x17 quadrant grids
     *
     * Working on the whole cell (instead of one quadrant) makes the Catmull-Rom result
     * identical for the shared rows/columns of neighboring quadrants, so quad seams line up
     * by construction.
     *
     * @param data Loaded land data holding heights[4][289]
     * @param grid Output grid, indexed [y][x] with x/y growing east/north
     */
    static void buildCellHeightGrid(const RE::TESObjectLAND::LoadedLandData& data,
                                    std::array<std::array<float,
                                                          K_CELL_DIM>,
                                               K_CELL_DIM>& grid);

    /**
     * @brief Samples the cell height grid with mirror extrapolation outside the cell
     *
     * Verts on a shared cell border only ever sample their own border line (see
     * sampleHeight), so the extrapolated samples never cause cross-cell seams.
     *
     * @param grid The 33x33 cell height grid
     * @param x Grid X, may be outside [0, 32]
     * @param y Grid Y, may be outside [0, 32]
     * @return float The (possibly extrapolated) height
     */
    static auto gridHeight(const std::array<std::array<float,
                                                       K_CELL_DIM>,
                                            K_CELL_DIM>& grid,
                           int x,
                           int y) -> float;

    /**
     * @brief Height at a fractional cell-grid position: Catmull-Rom blended with bilinear
     *
     * Separable Catmull-Rom collapses to the exact grid value at integer positions and to
     * line-only samples on grid lines, which keeps original verts fixed and makes both quad
     * and cell borders seam-free (both sides of a border interpolate from the same shared
     * line of samples).
     *
     * @param grid The 33x33 cell height grid
     * @param cellX Integer grid X of the containing coarse quad (0-32)
     * @param cellY Integer grid Y of the containing coarse quad (0-32)
     * @param fracX Fractional position within the coarse quad [0, 1)
     * @param fracY Fractional position within the coarse quad [0, 1)
     * @param smoothness 0 = bilinear only, 1 = full Catmull-Rom
     * @return float The interpolated height
     */
    static auto sampleHeight(const std::array<std::array<float,
                                                         K_CELL_DIM>,
                                              K_CELL_DIM>& grid,
                             int cellX,
                             int cellY,
                             float fracX,
                             float fracY,
                             float smoothness) -> float;

    /**
     * @brief 1D Catmull-Rom through p1 (t=0) and p2 (t=1)
     */
    static auto catmullRom(float p0,
                           float p1,
                           float p2,
                           float p3,
                           float t) -> float;

    /**
     * @brief Linear interpolation
     */
    static auto lerp(float valA,
                     float valB,
                     float t) -> float;

    /**
     * @brief Bilinear interpolation of four corner values
     */
    static auto bilerp(float v00,
                       float v10,
                       float v01,
                       float v11,
                       float fracX,
                       float fracY) -> float;

    /**
     * @brief Interpolates a full vertex record bilinearly between four coarse records
     *
     * Unit-length attributes (normal, tangent, bitangent) are decoded, lerped, renormalized
     * and re-encoded; everything else is lerped in its storage encoding. Position and height
     * are overwritten by the caller afterwards.
     *
     * @param c00 Record at (x0, y0)
     * @param c10 Record at (x1, y0)
     * @param c01 Record at (x0, y1)
     * @param c11 Record at (x1, y1)
     * @param fracX Weight toward c10/c11
     * @param fracY Weight toward c01/c11
     * @return LandVertex The interpolated record
     */
    static auto lerpVertex(const LandVertex& c00,
                           const LandVertex& c10,
                           const LandVertex& c01,
                           const LandVertex& c11,
                           float fracX,
                           float fracY) -> LandVertex;

    /**
     * @brief Returns (creating on first use) the engine index buffer for a subdivision level
     *
     * Mirrors the vanilla builder, which shares one static index buffer between every land
     * quad in the game: the triangulation only depends on the grid dimension, so one buffer
     * per level serves all quads. Never freed; each shape AddRefs the underlying D3D buffer.
     * The CPU copy the buffer was built from is kept in s_indexData.
     *
     * @param level Subdivision level 1-3
     * @return IndexBufferData* The shared wrapper, or nullptr when creation failed
     */
    static auto getIndexBuffer(int level) -> IndexBufferData*;

    /**
     * @brief Builds the index list of one subdivision level
     *
     * Same triangulation as the vanilla 17x17 builder, generalized to the finer grid: a
     * checkerboard of flipped diagonals with identical winding.
     *
     * @param level Subdivision level 1-3
     * @return std::vector<std::uint16_t> Three indices per triangle, in row major order
     */
    [[nodiscard]] static auto buildIndices(int level) -> std::vector<std::uint16_t>;

    /**
     * @brief Releases a replaced renderer data through the engine's geometry buffer manager
     *
     * Same call BSTriShape::~BSTriShape makes, keeping allocation and release symmetric
     * with the engine.
     */
    static void releaseRendererData(RE::BSGraphics::TriShape* rendererData);

    /**
     * @brief IEEE 754 binary16 -> binary32
     */
    static auto halfToFloat(std::uint16_t half) -> float;

    /**
     * @brief IEEE 754 binary32 -> binary16 (round to nearest even, matching the engine)
     */
    static auto floatToHalf(float value) -> std::uint16_t;

    /**
     * @brief Encodes a [-1, 1] component into the engine's unsigned byte encoding
     */
    static auto encodeUnitByte(float value) -> std::uint8_t;

    /**
     * @brief Decodes the engine's unsigned byte encoding back to [-1, 1]
     */
    static auto decodeUnitByte(std::uint8_t value) -> float;

    /**
     * @brief Rounds an interpolated byte attribute (colors, blend weights) back to storage
     */
    static auto roundByte(float value) -> std::uint8_t;
};

} // namespace SmoothTerrain
