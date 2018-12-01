#ifndef SHAWN_VECTOR_HPP
#define SHAWN_VECTOR_HPP

#include <vector>

namespace shawn {

class shawn_vector {
public:
    template <typename T1, typename T2>
    void unifySize(std::vector<T1> &p_vector1, std::vector<T2> &p_vector2) {
        if (p_vector1.size() == p_vector2.size()) {
            return;
        }
        if (p_vector1.size() > p_vector2.size()) {
            p_vector2.resize(p_vector1.size());
        } else {
            p_vector1.resize(p_vector2.size());
        }
    }
};

}

#endif //SHAWN_VECTOR_HPP