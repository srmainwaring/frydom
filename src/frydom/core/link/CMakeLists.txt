add_subdirectory(constraint)
add_subdirectory(links_lib)

target_sources(frydom PRIVATE
        FrLinkBase.cpp
        FrLinkMaskBase.cpp
        )