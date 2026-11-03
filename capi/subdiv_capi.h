//
//   Copyright 2025 Pixar
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

//
//   C-compatible (P/Invoke friendly) wrapper around the OpenSubdiv Far API.
//
//   Typical usage:
//       SubdivMesh* mesh = CreateMesh(nverts, positions, nfaces,
//                                     faceVertexCounts, faceVertexIndices);
//       SetCreases(mesh, ncreases, edgeVertexIndices, sharpness);
//       SetUVs(mesh, nuvs, uvs, faceUVIndices);
//       if (Subdivide(mesh, level)) {
//           int nverts = GetVertexCount(mesh);
//           GetPositions(mesh, positionsOut);
//           GetUVs(mesh, uvsOut);
//           GetFaceVertexCounts(mesh, countsOut);
//           GetFaceVertexIndices(mesh, indicesOut);
//       }
//       DestroyMesh(mesh);
//
//   All functions are exception-safe (no C++ exceptions cross this boundary)
//   and are NOT thread-safe for the same SubdivMesh instance.
//
#ifndef OPENSUBDIV_CAPI_SUBDIV_CAPI_H
#define OPENSUBDIV_CAPI_SUBDIV_CAPI_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(OSD_CAPI_EXPORTS)
#    define OSD_CAPI_EXPORT __declspec(dllexport)
#  else
#    define OSD_CAPI_EXPORT
#  endif
#else
#  if defined(OSD_CAPI_EXPORTS)
#    define OSD_CAPI_EXPORT __attribute__((visibility("default")))
#  else
#    define OSD_CAPI_EXPORT
#  endif
#endif

struct SubdivMesh;
typedef struct SubdivMesh SubdivMesh;

// Creates a mesh handle from raw topology and positions.
//   positions          : vertexCount * 3 floats (x,y,z per vertex)
//   faceVertexCounts   : faceCount ints, each >= 3
//   faceVertexIndices  : sum(faceVertexCounts) ints in [0, vertexCount)
// Returns NULL on invalid input. Creases and UVs may be attached afterwards.
OSD_CAPI_EXPORT SubdivMesh* CreateMesh(
    int vertexCount,
    const float* positions,
    int faceCount,
    const int* faceVertexCounts,
    const int* faceVertexIndices);

// Sets edge creases. edgeVertexIndices holds 2 vertex indices per crease
// (the endpoints of the edge), sharpness holds one float per crease.
// Must be called before Subdivide().
OSD_CAPI_EXPORT void SetCreases(
    SubdivMesh* mesh,
    int creaseCount,
    const int* edgeVertexIndices,
    const float* sharpness);

// Sets UVs as face-varying data. uvs holds uvCount * 2 floats (u,v per value).
// faceUVIndices holds one UV index per face corner, i.e.
// sum(faceVertexCounts) entries in [0, uvCount).
// Must be called before Subdivide().
OSD_CAPI_EXPORT void SetUVs(
    SubdivMesh* mesh,
    int uvCount,
    const float* uvs,
    const int* faceUVIndices);

// Uniformly subdivides the mesh to the given level (>= 1) using the
// Catmull-Clark scheme. Positions and UVs are interpolated to the last level.
// Returns true on success.
OSD_CAPI_EXPORT bool Subdivide(
    SubdivMesh* mesh,
    int subdivisionLevel);

// Number of vertices at the last refined level (0 if not subdivided).
OSD_CAPI_EXPORT int GetVertexCount(SubdivMesh* mesh);

// Copies GetVertexCount() * 3 floats (x,y,z per vertex) into output.
// output must have room for GetVertexCount() * 3 floats.
OSD_CAPI_EXPORT void GetPositions(
    SubdivMesh* mesh,
    float* output);

// Number of face-varying UV values at the last refined level
// (0 if not subdivided or if SetUVs() was not called).
OSD_CAPI_EXPORT int GetUVCount(SubdivMesh* mesh);

// Copies GetUVCount() * 2 floats (u,v per value) into output.
// output must have room for GetUVCount() * 2 floats.
OSD_CAPI_EXPORT void GetUVs(
    SubdivMesh* mesh,
    float* output);

// Number of faces at the last refined level (0 if not subdivided).
OSD_CAPI_EXPORT int GetFaceCount(SubdivMesh* mesh);

// Copies GetFaceCount() ints (vertices per face) into output.
// output must have room for GetFaceCount() ints.
OSD_CAPI_EXPORT void GetFaceVertexCounts(
    SubdivMesh* mesh,
    int* output);

// Copies the refined face vertex indices into output.
// output must have room for sum(GetFaceVertexCounts()) ints.
OSD_CAPI_EXPORT void GetFaceVertexIndices(
    SubdivMesh* mesh,
    int* output);

// Destroys a mesh handle previously returned by CreateMesh. NULL is allowed.
OSD_CAPI_EXPORT void DestroyMesh(SubdivMesh* mesh);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* OPENSUBDIV_CAPI_SUBDIV_CAPI_H */
