#include <CLI/CLI.hpp>
#include <glog/logging.h>

int main(int argc, char **argv) {
  CLI::App app{"Binary to PCD Converter"};
  argv = app.ensure_utf8(argv);

  std::string input_dir;
  app.add_option("-i,--input", input_dir, "Input directory")->required();

  std::string output_dir;
  app.add_option("-o,--output", output_dir, "Output directory")->required();

  CLI11_PARSE(app, argc, argv);

  LOG(INFO) << input_dir;
  LOG(INFO) << output_dir;
}