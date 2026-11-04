//
//   Copyright 2025 Pixar
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

#include "subdiv_capi.h"

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/primvarRefiner.h>

#include <cstring>
#include <memory>
#include <new>
#include <numeric>
#include <vector>

using namespace OpenSubdiv;

namespace {

// Minimal vertex primvar container required by PrimvarRefiner.
struct CapiPosition {

    CapiPosition() { }
    CapiPosition(CapiPosition const & src) {
        value[0] = src.value[0];
        value[1] = src.value[1];
        value[2] = src.value[2];
    }

    void Clear(void * = 0) {
        value[0] = value[1] = value[2] = 0.0f;
    }
    void AddWithWeight(CapiPosition const & src, float weight) {
        value[0] += weight * src.value[0];
        value[1] += weight * src.value[1];
        value[2] += weight * src.value[2];
    }

    float value[3];
};

// Minimal face-varying primvar container required by PrimvarRefiner.
struct CapiUV {

    CapiUV() { }
    CapiUV(CapiUV const & src) {
        value[0] = src.value[0];
        value[1] = src.value[1];
    }

    void Clear(void * = 0) {
        value[0] = value[1] = 0.0f;
    }
    void AddWithWeight(CapiUV const & src, float weight) {
        value[0] += weight * src.value[0];
        value[1] += weight * src.value[1];
    }

    float value[2];
};

// Generic per-vertex scalar primvar (interpolated like positions). The channel
// count is a runtime property of the mesh; unused channels stay zero.
struct CapiScalars {

    CapiScalars() {
        for (int i = 0; i < 16; ++i) value[i] = 0.0f;
    }
    CapiScalars(CapiScalars const & src) {
        for (int i = 0; i < 16; ++i) value[i] = src.value[i];
    }

    void Clear(void * = 0) {
        for (int i = 0; i < 16; ++i) value[i] = 0.0f;
    }
    void AddWithWeight(CapiScalars const & src, float weight) {
        for (int i = 0; i < 16; ++i) value[i] += weight * src.value[i];
    }

    float value[16];
};

} // anonymous namespace

//
//  The opaque handle exposed through the C API. Only referenced by pointer.
//
struct SubdivMesh {

    // Raw coarse inputs (owned copies of the caller's data).
    std::vector<float> coarsePositions;   // vertexCount * 3
    std::vector<int>   faceVertexCounts;  // faceCount
    std::vector<int>   faceVertexIndices; // sum(faceVertexCounts)
    std::vector<int>   creasePairs;       // creaseCount * 2
    std::vector<float> creaseSharpness;   // creaseCount
    std::vector<float> coarseUVs;         // uvCount * 2
    std::vector<int>   faceUVIndices;     // sum(faceVertexCounts)
    std::vector<float> coarseScalars;     // vertexCount * scalarChannels
    int                scalarChannels;
    int                fvarInterpolation; // Sdc::Options::FVarLinearInterpolation
    bool hasUVs;
    bool hasScalars;

    // Refined results (last level only).
    bool refined;
    int  level;
    std::vector<CapiPosition> positions;
    std::vector<CapiUV>       uvs;
    std::vector<float>        scalars;         // lastLevelVerts * scalarChannels
    std::vector<int>          faceCounts;
    std::vector<int>          faceIndices;
    std::vector<int>          refinedFaceUVIndices;  // sum(faceCounts)
    std::vector<int>          faceParentFaces;       // faceCount

    SubdivMesh()
        : scalarChannels(0), fvarInterpolation(0),
          hasUVs(false), hasScalars(false), refined(false), level(0) { }
};

namespace {

// True if the raw topology is self-consistent. Outputs the total number of
// face corners (sum of faceVertexCounts).
bool
validateTopology(int vertexCount, float const * positions,
                 int faceCount, int const * faceVertexCounts,
                 int const * faceVertexIndices, int * totalFaceVertices) {

    if (vertexCount <= 0 || !positions ||
        faceCount <= 0 || !faceVertexCounts || !faceVertexIndices) {
        return false;
    }

    long long total = 0;
    for (int face = 0; face < faceCount; ++face) {
        if (faceVertexCounts[face] < 3) {
            return false;
        }
        total += faceVertexCounts[face];
    }
    if (total > 0x7fffffffLL) {
        return false;
    }

    for (long long i = 0; i < total; ++i) {
        if (faceVertexIndices[i] < 0 || faceVertexIndices[i] >= vertexCount) {
            return false;
        }
    }

    *totalFaceVertices = (int) total;
    return true;
}

// Refines the mesh contents of 'mesh' to 'level' and stores the results of
// the last level back into it.
bool
refineInternal(SubdivMesh & mesh, int level) {

    if (level < 1 || mesh.refined) {
        return false;
    }

    // Build the topology descriptor referencing the mesh's owned copies.
    Far::TopologyDescriptor desc;   // zero-initialized by its constructor

    desc.numVertices        = (int) mesh.coarsePositions.size() / 3;
    desc.numFaces           = (int) mesh.faceVertexCounts.size();
    desc.numVertsPerFace    = mesh.faceVertexCounts.data();
    desc.vertIndicesPerFace = mesh.faceVertexIndices.data();

    desc.numCreases            = (int) mesh.creaseSharpness.size();
    desc.creaseVertexIndexPairs =
        mesh.creasePairs.empty() ? 0 : mesh.creasePairs.data();
    desc.creaseWeights =
        mesh.creaseSharpness.empty() ? 0 : mesh.creaseSharpness.data();

    desc.isLeftHanded = false;

    Far::TopologyDescriptor::FVarChannel fvarChannel;
    if (mesh.hasUVs) {
        fvarChannel.numValues    = (int) mesh.coarseUVs.size() / 2;
        fvarChannel.valueIndices = mesh.faceUVIndices.data();
        desc.numFVarChannels = 1;
        desc.fvarChannels    = &fvarChannel;
    }

    Sdc::SchemeType const scheme = Sdc::SCHEME_CATMARK;

    Sdc::Options options;
    options.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);
    options.SetFVarLinearInterpolation(
        (Sdc::Options::FVarLinearInterpolation) mesh.fvarInterpolation);

    std::unique_ptr<Far::TopologyRefiner> refiner(
        Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Create(
            desc, Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Options(
                scheme, options)));
    if (!refiner) {
        return false;
    }

    // Uniformly refine the topology up to 'level'.
    refiner->RefineUniform(Far::TopologyRefiner::UniformOptions(level));

    // Interpolate positions level by level.
    int const numTotalVerts = refiner->GetNumVerticesTotal();

    std::vector<CapiPosition> positions((size_t) numTotalVerts);
    for (int i = 0; i < desc.numVertices; ++i) {
        positions[(size_t) i].value[0] = mesh.coarsePositions[(size_t) i * 3 + 0];
        positions[(size_t) i].value[1] = mesh.coarsePositions[(size_t) i * 3 + 1];
        positions[(size_t) i].value[2] = mesh.coarsePositions[(size_t) i * 3 + 2];
    }

    // Interpolate UVs (face-varying primvar) level by level.
    std::vector<CapiUV> uvs;
    if (mesh.hasUVs) {
        uvs.resize((size_t) refiner->GetNumFVarValuesTotal(0));
        for (int i = 0; i < fvarChannel.numValues; ++i) {
            uvs[(size_t) i].value[0] = mesh.coarseUVs[(size_t) i * 2 + 0];
            uvs[(size_t) i].value[1] = mesh.coarseUVs[(size_t) i * 2 + 1];
        }
    }

    // Interpolate per-vertex scalar attributes level by level.
    int const scalarChannels = mesh.scalarChannels;
    std::vector<CapiScalars> scalars;
    if (mesh.hasScalars) {
        scalars.resize((size_t) numTotalVerts);
        for (int i = 0; i < desc.numVertices; ++i) {
            for (int c = 0; c < scalarChannels; ++c) {
                scalars[(size_t) i].value[c] =
                    mesh.coarseScalars[(size_t) i * scalarChannels + c];
            }
        }
    }

    Far::PrimvarRefiner primvarRefiner(*refiner);

    CapiPosition * srcPos = &positions[0];
    CapiUV *       srcUV  = mesh.hasUVs ? &uvs[0] : 0;
    CapiScalars *  srcScl = mesh.hasScalars ? &scalars[0] : 0;

    for (int current = 1; current <= level; ++current) {
        int const numPrevVerts = refiner->GetLevel(current - 1).GetNumVertices();

        CapiPosition * dstPos = srcPos + numPrevVerts;
        primvarRefiner.Interpolate(current, srcPos, dstPos);
        srcPos = dstPos;

        if (mesh.hasScalars) {
            CapiScalars * dstScl = srcScl + numPrevVerts;
            primvarRefiner.Interpolate(current, srcScl, dstScl);
            srcScl = dstScl;
        }

        if (mesh.hasUVs) {
            int const numPrevValues =
                refiner->GetLevel(current - 1).GetNumFVarValues(0);

            CapiUV * dstUV = srcUV + numPrevValues;
            primvarRefiner.InterpolateFaceVarying(current, srcUV, dstUV, 0);
            srcUV = dstUV;
        }
    }

    // Copy the topology and primvar data of the last level into the mesh.
    Far::TopologyLevel const & lastLevel = refiner->GetLevel(level);

    int const numLastVerts    = lastLevel.GetNumVertices();
    int const offsetLastVerts = numTotalVerts - numLastVerts;

    mesh.positions.assign(positions.begin() + offsetLastVerts, positions.end());

    if (mesh.hasScalars) {
        int const numLastVerts = lastLevel.GetNumVertices();
        size_t const offsetLastScalars = positions.size() - numLastVerts;

        mesh.scalars.reserve((size_t) numLastVerts * scalarChannels);
        for (size_t i = offsetLastScalars; i < scalars.size(); ++i) {
            for (int c = 0; c < scalarChannels; ++c) {
                mesh.scalars.push_back(scalars[i].value[c]);
            }
        }
    }

    if (mesh.hasUVs) {
        int const numLastValues    = lastLevel.GetNumFVarValues(0);
        int const offsetLastValues = (int) uvs.size() - numLastValues;
        mesh.uvs.assign(uvs.begin() + offsetLastValues, uvs.end());
    }

    mesh.faceCounts.clear();
    mesh.faceIndices.clear();
    mesh.refinedFaceUVIndices.clear();
    mesh.faceParentFaces.clear();

    int const numFaces = lastLevel.GetNumFaces();
    mesh.faceCounts.reserve((size_t) numFaces);
    for (int face = 0; face < numFaces; ++face) {
        Far::ConstIndexArray faceVerts = lastLevel.GetFaceVertices(face);
        mesh.faceCounts.push_back(faceVerts.size());
        for (int i = 0; i < faceVerts.size(); ++i) {
            mesh.faceIndices.push_back(faceVerts[i]);
        }

        if (mesh.hasUVs) {
            Far::ConstIndexArray faceUVs =
                lastLevel.GetFaceFVarValues(face, 0);
            for (int i = 0; i < faceUVs.size(); ++i) {
                mesh.refinedFaceUVIndices.push_back(faceUVs[i]);
            }
        }

        // GetFaceParentFace maps into the previous level; resolve the chain
        // up to the coarse (level 0) face.
        int parent = lastLevel.GetFaceParentFace(face);
        for (int current = level - 1; current >= 1; --current) {
            parent = refiner->GetLevel(current).GetFaceParentFace(parent);
        }
        mesh.faceParentFaces.push_back(parent);
    }

    mesh.refined = true;
    mesh.level   = level;

    return true;
}

} // anonymous namespace

extern "C" {

OSD_CAPI_EXPORT SubdivMesh*
CreateMesh(int vertexCount, const float* positions,
           int faceCount, const int* faceVertexCounts,
           const int* faceVertexIndices) {

    int totalFaceVertices = 0;
    if (!validateTopology(vertexCount, positions, faceCount,
                          faceVertexCounts, faceVertexIndices,
                          &totalFaceVertices)) {
        return 0;
    }

    try {
        std::unique_ptr<SubdivMesh> mesh(new SubdivMesh);

        mesh->coarsePositions.assign(positions,
                                     positions + (size_t) vertexCount * 3);
        mesh->faceVertexCounts.assign(faceVertexCounts,
                                      faceVertexCounts + (size_t) faceCount);
        mesh->faceVertexIndices.assign(faceVertexIndices,
                                       faceVertexIndices + (size_t) totalFaceVertices);
        return mesh.release();

    } catch (std::bad_alloc const &) {
        return 0;
    } catch (...) {
        return 0;
    }
}

OSD_CAPI_EXPORT void
SetCreases(SubdivMesh* mesh, int creaseCount,
           const int* edgeVertexIndices, const float* sharpness) {

    if (!mesh || mesh->refined) {
        return;
    }
    if (creaseCount < 0 ||
        (creaseCount > 0 && (!edgeVertexIndices || !sharpness))) {
        return;
    }

    int const numVertices = (int) mesh->coarsePositions.size() / 3;
    for (int i = 0; i < creaseCount * 2; ++i) {
        if (edgeVertexIndices[i] < 0 || edgeVertexIndices[i] >= numVertices) {
            return;
        }
    }

    try {
        mesh->creasePairs.assign(edgeVertexIndices,
                                 edgeVertexIndices + (size_t) creaseCount * 2);
        mesh->creaseSharpness.assign(sharpness,
                                     sharpness + (size_t) creaseCount);
    } catch (...) {
        mesh->creasePairs.clear();
        mesh->creaseSharpness.clear();
    }
}

OSD_CAPI_EXPORT void
SetUVs(SubdivMesh* mesh, int uvCount,
       const float* uvs, const int* faceUVIndices) {

    if (!mesh || mesh->refined) {
        return;
    }

    int const totalFaceVertices = (int) mesh->faceVertexIndices.size();
    if (uvCount <= 0 || !uvs || !faceUVIndices) {
        return;
    }
    for (int i = 0; i < totalFaceVertices; ++i) {
        if (faceUVIndices[i] < 0 || faceUVIndices[i] >= uvCount) {
            return;
        }
    }

    try {
        mesh->coarseUVs.assign(uvs, uvs + (size_t) uvCount * 2);
        mesh->faceUVIndices.assign(faceUVIndices,
                                   faceUVIndices + (size_t) totalFaceVertices);
        mesh->hasUVs = true;
    } catch (...) {
        mesh->coarseUVs.clear();
        mesh->faceUVIndices.clear();
        mesh->hasUVs = false;
    }
}

OSD_CAPI_EXPORT void
SetVertexScalars(SubdivMesh* mesh, int channelsPerVertex,
                 const float* scalars) {

    if (!mesh || mesh->refined) {
        return;
    }
    if (channelsPerVertex < 1 || channelsPerVertex > 16 || !scalars) {
        return;
    }

    int const numVertices = (int) mesh->coarsePositions.size() / 3;

    try {
        mesh->coarseScalars.assign(scalars,
                                   scalars + (size_t) numVertices * channelsPerVertex);
        mesh->scalarChannels = channelsPerVertex;
        mesh->hasScalars = true;
    } catch (...) {
        mesh->coarseScalars.clear();
        mesh->scalarChannels = 0;
        mesh->hasScalars = false;
    }
}

OSD_CAPI_EXPORT void
SetFVarLinearInterpolation(SubdivMesh* mesh, int mode) {

    if (!mesh || mesh->refined) {
        return;
    }
    if (mode < 0 || mode > 5) {
        return;
    }

    mesh->fvarInterpolation = mode;
}

OSD_CAPI_EXPORT bool
Subdivide(SubdivMesh* mesh, int subdivisionLevel) {

    if (!mesh) {
        return false;
    }
    try {
        return refineInternal(*mesh, subdivisionLevel);
    } catch (...) {
        return false;
    }
}

OSD_CAPI_EXPORT int
GetVertexCount(SubdivMesh* mesh) {

    return (mesh && mesh->refined) ? (int) mesh->positions.size() : 0;
}

OSD_CAPI_EXPORT void
GetPositions(SubdivMesh* mesh, float* output) {

    if (mesh && mesh->refined && output) {
        std::memcpy(output, &mesh->positions[0],
                    mesh->positions.size() * 3 * sizeof(float));
    }
}

OSD_CAPI_EXPORT int
GetUVCount(SubdivMesh* mesh) {

    return (mesh && mesh->refined && mesh->hasUVs) ?
        (int) mesh->uvs.size() : 0;
}

OSD_CAPI_EXPORT void
GetUVs(SubdivMesh* mesh, float* output) {

    if (mesh && mesh->refined && mesh->hasUVs && output) {
        std::memcpy(output, &mesh->uvs[0],
                    mesh->uvs.size() * 2 * sizeof(float));
    }
}

OSD_CAPI_EXPORT int
GetScalarCount(SubdivMesh* mesh) {

    // The scalar buffer already stores all channels interleaved.
    return (mesh && mesh->refined && mesh->hasScalars) ?
        (int) mesh->scalars.size() : 0;
}

OSD_CAPI_EXPORT void
GetScalars(SubdivMesh* mesh, float* output) {

    if (mesh && mesh->refined && mesh->hasScalars && output) {
        std::memcpy(output, &mesh->scalars[0],
                    mesh->scalars.size() * sizeof(float));
    }
}

OSD_CAPI_EXPORT int
GetFaceCount(SubdivMesh* mesh) {

    return (mesh && mesh->refined) ? (int) mesh->faceCounts.size() : 0;
}

OSD_CAPI_EXPORT void
GetFaceVertexCounts(SubdivMesh* mesh, int* output) {

    if (mesh && mesh->refined && output) {
        std::memcpy(output, &mesh->faceCounts[0],
                    mesh->faceCounts.size() * sizeof(int));
    }
}

OSD_CAPI_EXPORT void
GetFaceVertexIndices(SubdivMesh* mesh, int* output) {

    if (mesh && mesh->refined && output) {
        std::memcpy(output, &mesh->faceIndices[0],
                    mesh->faceIndices.size() * sizeof(int));
    }
}

OSD_CAPI_EXPORT void
GetFaceUVIndices(SubdivMesh* mesh, int* output) {

    if (!mesh || !mesh->refined || !output) {
        return;
    }

    if (mesh->hasUVs) {
        std::memcpy(output, &mesh->refinedFaceUVIndices[0],
                    mesh->refinedFaceUVIndices.size() * sizeof(int));
    } else {
        int const total = std::accumulate(mesh->faceCounts.begin(),
                                          mesh->faceCounts.end(), 0);
        for (int i = 0; i < total; ++i) {
            output[i] = -1;
        }
    }
}

OSD_CAPI_EXPORT void
GetFaceParentFaces(SubdivMesh* mesh, int* output) {

    if (mesh && mesh->refined && output) {
        std::memcpy(output, &mesh->faceParentFaces[0],
                    mesh->faceParentFaces.size() * sizeof(int));
    }
}

OSD_CAPI_EXPORT void
DestroyMesh(SubdivMesh* mesh) {

    delete mesh;
}

} // extern "C"
