#ifndef SRC_POLICY_H_
#define SRC_POLICY_H_
#include <onnx.h>

struct actor{
	struct onnx_context_t* ctx;
	struct onnx_tensor_t* input;
	struct onnx_tensor_t* output;
};

void alloc_actor_ctx(struct actor* act);
void free_actor_ctx(struct actor* act);
int get_action(struct actor* act, float* state, const size_t sizeof_state, const int action_len);

#endif
