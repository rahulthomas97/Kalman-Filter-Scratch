#pragma once
#include <eigen3/Eigen/Dense>
#include <toml11/parser.hpp>
#include <toml11/find.hpp>

using matd = Eigen::MatrixXd;
using vecd = Eigen::VectorXd;
class TomlParser
{
public:
    TomlParser(std::string s);
    matd getMatrixFromToml(std::string name);
    vecd getVectorFromToml(std::string name);
    template<typename Scalar>
    Scalar getValFromToml(std::string name)
    {
        Scalar x = toml::find<Scalar>(config, name);
        return x;
    }

private:
    std::string filename;
    toml::value config;
};
