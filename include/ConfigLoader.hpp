#pragma once

#include <cstddef>
#include <filesystem>

namespace SmoothTerrain {

/**
 * @brief Loads and serves the plugin configuration from Data/SKSE/Plugins/SmoothTerrain.ini
 *
 * The configuration is read once at plugin load (loadConfig) into a static ConfigMap; the
 * getters are plain accessors and never touch the disk. Missing files, missing keys, or
 * unparsable values silently fall back to the compiled-in defaults.
 */
class ConfigLoader {
private:
    //
    // DEFAULT CFG VALUES
    //
    constexpr static float DEFAULT_SUBDIVISIONS = 1.0F; /**< Default subdivision level (iSubdivisions) */
    constexpr static float DEFAULT_SMOOTHNESS = 1.0F; /**< Default interpolation smoothness (fSmoothness) */

public:
    constexpr static int MAX_SUBDIVISIONS = 3; /**< Hard cap: level 4 would overflow BSTriShape's 16-bit vertex count
                                                  (129x129 = 16641 verts at level 3 is the last that fits) */

private:
    /**
     * @brief ConfigMap structure which holds the configuration values for the plugin
     */
    struct ConfigMap {
        int subdivisions {}; /**< How many times each 128-unit land quad is split in half per axis (0-3) */
        float smoothness {}; /**< 0 = flat bilinear interpolation, 1 = full Catmull-Rom smoothing */
    };

    static inline ConfigMap s_config; /**< Holds the current configuration values for the plugin */

    //
    // Hardcoded Settings
    //
    constexpr static size_t INI_BUFFER_SIZE = 64; /**< Character buffer size for reading a single INI value */

public:
    /**
     * @brief Loads the configuration values from the SmoothTerrain.ini file and stores them in s_config
     */
    static void loadConfig();

    /**
     * @brief Get the subdivision level
     *
     * @return int Number of times each land quad edge is halved (0 = vanilla mesh, max 3)
     */
    static auto getSubdivisions() -> int;

    /**
     * @brief Get the interpolation smoothness
     *
     * @return float 0 keeps interpolated verts on the vanilla (flat-shaded) surface, 1 applies full
     *         Catmull-Rom curvature between the original verts
     */
    static auto getSmoothness() -> float;

private:
    /**
     * @brief Reads a single float value from the [General] section of an INI file
     *
     * @param path Path to the INI file
     * @param key Name of the key to read
     * @param defVal Value to return when the file or key is missing, or the value does not parse
     * @return float The parsed value, or defVal on any failure
     */
    static auto readIniFloat(const std::filesystem::path& path,
                             const wchar_t* key,
                             float defVal) -> float;
};

} // namespace SmoothTerrain
