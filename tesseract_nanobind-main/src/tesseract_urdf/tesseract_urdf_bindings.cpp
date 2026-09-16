/**
 * @file tesseract_urdf_bindings.cpp
 * @brief nanobind bindings for tesseract_urdf
 */

#include "tesseract_nb.h"

#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/resource_locator.h>

namespace tu = tesseract::urdf;

NB_MODULE(_tesseract_urdf, m) {
    m.doc() = "tesseract_urdf Python bindings";

    // Cross-module type registration: parseURDF* return SceneGraph (by value),
    // whose nb::class_ lives in _tesseract_scene_graph. Import it so the type is
    // registered even when the caller never imported tesseract_scene_graph —
    // otherwise the return-value cast fails with "Unable to convert function
    // return value". Latent since day one: the full test suite always imported
    // scene_graph during collection; the minimal-import wheel canary exposed it.
    nb::module_::import_("tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph");
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // parseURDFString - returns unique_ptr, nanobind handles conversion
    m.def("parseURDFString", &tu::parseURDFString,
          "urdf_xml_string"_a, "locator"_a,
          "Parse a URDF string into a SceneGraph");

    // parseURDFFile
    m.def("parseURDFFile", &tu::parseURDFFile,
          "path"_a, "locator"_a,
          "Parse a URDF file into a SceneGraph");

    // writeURDFFile
    m.def("writeURDFFile", &tu::writeURDFFile,
          "scene_graph"_a, "package_path"_a, "urdf_name"_a = "",
          "Write a SceneGraph to a URDF file");
}
