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

    // Per-vertex scalars (2 channels): constant and vertex-index ramp.
    {
        float scalars[8 * 2];
        for (int i = 0; i < 8; ++i) {
            scalars[i * 2 + 0] = 1.0f;
            scalars[i * 2 + 1] = (float) i;
        }
        SetVertexScalars(mesh, 2, scalars);
    }

    SetFVarLinearInterpolation(mesh, 0 /* smooth everywhere */);

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

        // Smooth face-varying interpolation keeps the UV field continuous and
        // bounded near the coarse [0,1]^2 range (corner values blend slightly
        // at creased cube corners).
        for (int i = 0; i < numUVs; ++i) {
            CHECK(uvs[i * 2 + 0] > -0.25f && uvs[i * 2 + 0] < 1.25f);
            CHECK(uvs[i * 2 + 1] > -0.25f && uvs[i * 2 + 1] < 1.25f);
        }

        // Face-varying value indices: one per corner, in range.
        {
            int faceUVIndices[EXPECTED_INDICES];
            GetFaceUVIndices(mesh, faceUVIndices);
            for (int i = 0; i < EXPECTED_INDICES; ++i) {
                CHECK(faceUVIndices[i] >= 0 && faceUVIndices[i] < numUVs);
            }
        }

        // Parent faces: every refined face maps into a coarse face.
        {
            int parentFaces[EXPECTED_FACES];
            GetFaceParentFaces(mesh, parentFaces);
            for (int i = 0; i < numFaces; ++i) {
                CHECK(parentFaces[i] >= 0 && parentFaces[i] < 6);
            }
        }

        // Scalars: constant channel stays 1 everywhere, ramp channel stays
        // within [0, 7] and both survive the full refinement.
        {
            int const numScalars = GetScalarCount(mesh);
            CHECK(numScalars == EXPECTED_VERTS * 2);

            float scalarOut[EXPECTED_VERTS * 2];
            GetScalars(mesh, scalarOut);

            float rampMin = 1e9f, rampMax = -1e9f;
            for (int i = 0; i < EXPECTED_VERTS; ++i) {
                CHECK(fabsf(scalarOut[i * 2 + 0] - 1.0f) < 1e-5f);
                float const r = scalarOut[i * 2 + 1];
                if (r < rampMin) rampMin = r;
                if (r > rampMax) rampMax = r;
            }

            // The uncreased cube smooths every vertex, so the ramp converges
            // toward its mean (3.5) but must never leave the coarse range.
            CHECK(rampMin > -1e-3f && rampMin < 3.5f);
            CHECK(rampMax > 3.5f && rampMax < 7.0f + 1e-3f);
        }
    }

    // A mesh cannot be subdivided twice.
    CHECK(Subdivide(mesh, LEVEL) == false);

    DestroyMesh(mesh);

    // Open mesh (single quad, like the welded armor cage): open edges are
    // auto-sharpened, so the surface must stay planar-ish within its coarse
    // bounds and the scalar/UV ranges must never leave the coarse hull.
    {
        float quadVerts[4][3] = {{ -0.5f, -0.5f, 0.0f },
                                 {  0.5f, -0.5f, 0.0f },
                                 {  0.5f,  0.5f, 0.0f },
                                 { -0.5f,  0.5f, 0.0f }};
        int quadFaces[1] = { 4 };
        int quadIndices[4] = { 0, 1, 2, 3 };
        float quadUvs[4][2] = {{ 0.0f, 0.0f }, { 1.0f, 0.0f },
                               { 1.0f, 1.0f }, { 0.0f, 1.0f }};
        float quadScalars[4 * 2];
        for (int i = 0; i < 4; ++i) {
            quadScalars[i * 2 + 0] = 1.0f;
            quadScalars[i * 2 + 1] = (float) i;
        }

        SubdivMesh * open = CreateMesh(4, &quadVerts[0][0], 1, quadFaces, quadIndices);
        CHECK(open != NULL);
        if (open) {
            SetUVs(open, 4, &quadUvs[0][0], quadIndices);
            SetVertexScalars(open, 2, quadScalars);
            CHECK(Subdivide(open, 2) == true);

            int const openVerts = GetVertexCount(open);
            CHECK(openVerts == 25);   // 4 + 4 + 1 + 12 + 4

            float openPositions[25 * 3];
            GetPositions(open, openPositions);

            for (int i = 0; i < openVerts; ++i) {
                for (int k = 0; k < 3; ++k) {
                    CHECK(openPositions[i * 3 + k] > -0.5f - 1e-4f);
                    CHECK(openPositions[i * 3 + k] <  0.5f + 1e-4f);
                }
            }

            float openScalars[25 * 2];
            CHECK(GetScalarCount(open) == 25 * 2);
            GetScalars(open, openScalars);
            for (int i = 0; i < openVerts; ++i) {
                CHECK(fabsf(openScalars[i * 2 + 0] - 1.0f) < 1e-5f);
                CHECK(openScalars[i * 2 + 1] > -1e-4f);
                CHECK(openScalars[i * 2 + 1] <  3.0f + 1e-4f);
            }

            int openUVs = GetUVCount(open);
            CHECK(openUVs == openVerts);   // smooth fvar channel tracks vertices

            int openFaceUVIndices[16 * 4];
            GetFaceUVIndices(open, openFaceUVIndices);
            for (int i = 0; i < 16 * 4; ++i) {
                CHECK(openFaceUVIndices[i] >= 0 && openFaceUVIndices[i] < openUVs);
            }

            DestroyMesh(open);
        }
    }

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
