#include "../../inc/pinocchio/pinocchio_interface.h"

class PinocchioDummy : public PinocchioInterface
{
public:

    PinocchioDummy() {}

    vector_t f(const vector_t &q, const vector_t &v, const vector_t &a, const domain &d) override
    {
        vector_t none(0);
        return none;
    };

    void Df(const vector_t q, const vector_t v, const vector_t a, const domain d,
                              matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) override
    {
       return;
    };

    vector_t delta_f(const vector_t q, const vector_t v, const domain d) override
    {
        vector_t none(0);
        return none;
    };

    void Ddelta_f(const vector_t q, const vector_t v, const domain d,
                                    matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) override
    {
        return;
    }
};

std::unique_ptr<PinocchioInterface> createPinocchioInstance()
{
    return std::make_unique<PinocchioDummy>();
}