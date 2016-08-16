/*
	Author: Martin Schröder <mkschreder.uk@gmail.com>

	Check compliance of this library with GLM implementation.
*/

#define GLM_FORCE_RADIANS

#include <stdio.h>
#include <math.h>
#include <cfloat>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../matrix/math.hpp"
#include "test_macros.hpp"

using namespace matrix;

int main() {
    glm::quat gq[3] = {
        glm::quat(cos(glm::radians(90.0f/2.0f)), sin(glm::radians(90.0f/2.0f)), 0, 0),
        glm::quat(cos(glm::radians(90.0f/2.0f)), 0, sin(glm::radians(90.0f/2.0f)), 0),
        glm::quat(cos(glm::radians(90.0f/2.0f)), 0, 0, sin(glm::radians(90.0f/2.0f)))
    };

    glm::vec3 gv[3] = {
        glm::vec3(1, 0, 0),
        glm::vec3(0, 1, 0),
        glm::vec3(0, 0, 1)
    };

    matrix::Quatf mq[3] = {
        matrix::Quatf(cos(glm::radians(90.0f/2.0f)), sin(glm::radians(90.0f/2.0f)), 0, 0),
        matrix::Quatf(cos(glm::radians(90.0f/2.0f)), 0, sin(glm::radians(90.0f/2.0f)), 0),
        matrix::Quatf(cos(glm::radians(90.0f/2.0f)), 0, 0, sin(glm::radians(90.0f/2.0f)))
    };

    matrix::Vector3f mv[3] = {
        matrix::Vector3f(1, 0, 0),
        matrix::Vector3f(0, 1, 0),
        matrix::Vector3f(0, 0, 1)
    };

    for(int c = 0; c < 3; c++) {
        printf("testing variation %d\n", c);
        for(int j = 0; j < 3; j++) {
            printf("\tvector %f %f %f\n", gv[j].x, gv[j].y, gv[j].z);
            glm::vec3 gr = gq[c] * gv[j];
            matrix::Vector3f mr = mq[c] * mv[j];
            TEST(is_equal(gr.x, mr(0)) && is_equal(gr.y,  mr(1)) && is_equal(gr.z, mr(2)));
        }
    }
}
