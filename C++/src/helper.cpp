#include <helper.h>

TomlParser::TomlParser(std::string s)
    : filename(s)
{
    config = toml::parse(filename);
}

matd TomlParser::getMatrixFromToml(std::string name)
{
    matd mat;
    std::vector<std::vector<double>> M = toml::find<std::vector<std::vector<double>>>(config, name);
    int rows = M.size();
    int cols = M[0].size();
    mat.resize(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = M[i][j];
        }
    }

    return mat;
}

vecd TomlParser::getVectorFromToml(std::string name)
{
    vecd vec;
    std::vector<double> V = toml::find<std::vector<double>>(config, name);
    vec.resize(V.size());
    for (int i = 0; i < V.size(); i++) {
        vec(i) = V[i];
    }
    return vec;
}
