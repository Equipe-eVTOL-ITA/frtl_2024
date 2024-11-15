#ifndef BASE_CPP
#define BASE_CPP

struct Base{
    Eigen::Vector3d coordinates;
    std::string name;
    bool is_visited = false;
};

#endif