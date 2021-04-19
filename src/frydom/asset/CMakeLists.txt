message(STATUS "    ...Asset")

add_subdirectory(shape)

target_sources(frydom PRIVATE
        FrAsset.cpp
        FrAssetOwner.cpp
        FrNodeAsset.cpp
        FrForceAsset.cpp
        FrGridAsset.cpp
        FrSeabedGridAsset.cpp
        FrFreeSurfaceGridAsset.cpp
        FrCatenaryLineAsset.cpp
        )
