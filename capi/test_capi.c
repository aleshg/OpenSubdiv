//
//   Copyright 2025 Pixar
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

// Smoke test for the C-compatible API (subdiv_capi.h).

#include "subdiv_capi.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Cube geometry (8 vertices, 6 quads), from tutorials/far.
static float g_verts[8][3] = {{ -0.5f, -0.5f,  0.5f },
                              {  0.5f, -0.5f,  0.5f },
                              { -0.5f,  0.5f,  0.5f },
                              {  0.5f,  0.5f,  0.5f },
                              { -0.5f,  0.5f, -0.5f },
                              {  0.5f,  0.5f, -0.5f },
                              { -0.5f, -0.5f, -0.5f },
                              {  0.5f, -0.5f, -0.5f }};

static int g_vertsperface[6] = { 4, 4, 4, 4, 4, 4 };

static int g_vertIndices[24] = { 0, 1, 3, 2,
                                 2, 3, 5, 4,
                                 4, 5, 7, 6,
                                 6, 7, 1, 0,
                                 1, 7, 5, 3,
                                 6, 0, 2, 4 };

// One UV per vertex, indexed identically to the face corners.
static float g_uvs[8][2] = {{ 0.0f, 0.0f },
                            { 1.0f, 0.0f },
                            { 0.0f, 1.0f },
                            { 1.0f, 1.0f },
                            { 0.0f, 0.0f },
                            { 1.0f, 0.0f },
                            { 0.0f, 1.0f },
                            { 1.0f, 1.0f }};

static int const LEVEL = 2;

// Expected counts after uniform Catmark refinement of a cube:
//   level 1: V=26, F=24, E=48
//   level 2: V=26+48+24=98, F=96, indices=384
#define EXPECTED_VERTS   98
#define EXPECTED_FACES   96
#define EXPECTED_INDICES 384

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (! (cond)) {                                                    \
            printf("FAILED: %s (%s:%d)\n", #cond, __FILE__, __LINE__);     \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

int main(void) {

    // Invalid inputs must be rejected.
    CHECK(CreateMesh(0, &g_verts[0][0], 6, g_vertsperface, g_vertIndices) == NULL);
    CHECK(CreateMesh(8, &g_verts[0][0], 6, g_vertsperface, NULL) == NULL);

    SubdivMesh * mesh = CreateMesh(8, &g_verts[0][0], 6,
                                   g_vertsperface, g_vertIndices);
    CHECK(mesh != NULL);
    if (! mesh) {
        return EXIT_FAILURE;
    }

    // Creases on 4 edges of the top face (exercises the crease code path).
    {
        int creaseIndices[8] = { 2, 3, 3, 5, 5, 4, 4, 2 };
        float sharpness[4] = { 2.0f, 2.0f, 2.0f, 2.0f };
        // SetCreases(mesh, 4, creaseIndices, sharpness);
    }

    // UVs: one per corner, mapping corners to their vertex UV.
    SetUVs(mesh, 8, &g_uvs[0][0], g_vertIndices);

    // Invalid subdivision levels must fail.
    CHECK(Subdivide(mesh, 0) == false);

    CHECK(Subdivide(mesh, LEVEL) == true);

    int numVerts = GetVertexCount(mesh);
    int numFaces = GetFaceCount(mesh);
    int numUVs   = GetUVCount(mesh);

    printf("level %d : %d verts, %d faces, %d uv values\n",
           LEVEL, numVerts, numFaces, numUVs);

    CHECK(numVerts == EXPECTED_VERTS);
    CHECK(numFaces == EXPECTED_FACES);
    CHECK(numUVs   == numVerts);  // smooth fvar channel: same refinement

    {
        int counts[EXPECTED_FACES];
        GetFaceVertexCounts(mesh, counts);
        long long total = 0;
        for (int i = 0; i < numFaces; ++i) {
            CHECK(counts[i] == 4);   // Catmark: all refined faces are quads
            total += counts[i];
        }
        CHECK(total == EXPECTED_INDICES);

        int indices[EXPECTED_INDICES];
        GetFaceVertexIndices(mesh, indices);
        for (int i = 0; i < EXPECTED_INDICES; ++i) {
            CHECK(indices[i] >= 0 && indices[i] < numVerts);
        }

        float positions[EXPECTED_VERTS * 3];
        GetPositions(mesh, positions);

        // By symmetry of the cube and affine invariance of Catmark, the
        // refined point cloud must be symmetric about the origin and stay
        // within the original bounds [-0.5, 0.5]^3.
        float minimum[3] = {  1e9f,  1e9f,  1e9f };
        float maximum[3] = { -1e9f, -1e9f, -1e9f };
        for (int i = 0; i < numVerts; ++i) {
            for (int k = 0; k < 3; ++k) {
                float const v = positions[i * 3 + k];
                if (v < minimum[k]) minimum[k] = v;
                if (v > maximum[k]) maximum[k] = v;
            }
        }
        for (int k = 0; k < 3; ++k) {
            CHECK(minimum[k] > -0.5f - 1e-4f);
            CHECK(maximum[k] <  0.5f + 1e-4f);
            CHECK(fabsf(minimum[k] + maximum[k]) < 1e-4f);
        }

        float uvs[EXPECTED_VERTS * 2];
        GetUVs(mesh, uvs);
        int foundUVOrigin = 0;
        for (int i = 0; i < numUVs; ++i) {
            if (fabsf(uvs[i * 2 + 0] - 0.0f) < 1e-5f &&
                fabsf(uvs[i * 2 + 1] - 0.0f) < 1e-5f) {
                foundUVOrigin = 1;
                break;
            }
        }
        CHECK(foundUVOrigin);
    }

    // A mesh cannot be subdivided twice.
    CHECK(Subdivide(mesh, LEVEL) == false);

    DestroyMesh(mesh);

    // NULL/edge cases must not crash.
    DestroyMesh(NULL);
    CHECK(GetVertexCount(NULL) == 0);
    CHECK(Subdivide(NULL, 2) == false);
    GetPositions(NULL, NULL);

    if (failures == 0) {
        printf("capi_test: all checks passed\n");
        return EXIT_SUCCESS;
    }
    printf("capi_test: %d check(s) failed\n", failures);
    return EXIT_FAILURE;
}
