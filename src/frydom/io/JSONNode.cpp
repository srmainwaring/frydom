#include "JSONNode.h"

namespace frydom {

  namespace internal {

    void walk_over_node(const json &node, const fs::path &current_root_dir, json &working_node) {

      for (auto it = node.cbegin(); it != node.cend(); ++it) {

        std::string key = it.key();

        if (is_comment(key)) {
          working_node.erase(key);
          continue;
        }

        // node[key] is not a leaf
        if (it->is_object()) {

//      std::cout << "Structured: " << key << std::endl;

          if (exists(*it, "json_file")) {
            // We only read this field !

            std::string json_filename = (*it)["json_file"];
//        std::cout << "JSON file: " << json_filename << std::endl;

            json_filename = current_root_dir / json_filename;
            if (!fs::exists(json_filename)) {
              std::cerr << "File " << json_filename << " not found" << std::endl;
              exit(EXIT_FAILURE);
            }

            json_filename = fs::canonical(current_root_dir / json_filename);

            std::ifstream ifs(json_filename);
            json nested_node = json::parse(ifs);

            if (!exists(nested_node, key)) {
              std::cerr << "Nested file " << json_filename << " should have the key " << key << " at its root"
                        << std::endl;
              exit(EXIT_FAILURE);
            }

            working_node[key] = nested_node[key];

            walk_over_node(nested_node[key], fs::path(json_filename).remove_filename(),
                           working_node[key]); // TODO: changer root_dir

          } else {
            walk_over_node(node[key], current_root_dir, working_node[key]);
          }

        } else {
          // This is a leaf

          if (key == "json_file") {
            std::cerr << "In JONNODE.cpp: It should not be possible to reach this point... Something went wrong. Abort"
                      << std::endl;
            exit(EXIT_FAILURE);

          } else {
//        std::cout << "Leaf value: " << key << std::endl;

            if (startswith(key, "file_")) {

              std::string file_path = it.value();
              file_path = fs::canonical(current_root_dir / file_path);

              if (!fs::exists(file_path)) {
                std::cerr << "Referenced file " << file_path << " does not exists" << std::endl;
                exit(EXIT_FAILURE);
              }

//          std::cout << "\tFile " << file_path << std::endl;
              working_node.erase(key);

              key.erase(0, 5);

              working_node[key] = file_path;
            }

          }
        }

      }
    }


  }  // end namespace frydom::internal
}  // end namespace frydom