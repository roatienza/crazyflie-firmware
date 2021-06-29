#include <onnx.h>
#include "actor.cc"
#include "debug.h"
#include "policy.h"

void alloc_actor_ctx(struct actor* act)
{
    act->ctx = onnx_context_alloc(actor_onnx, sizeof(actor_onnx), NULL, 0);
}

void free_actor_ctx(struct actor* act)
{
	onnx_context_free(act->ctx);
}

int get_action(struct actor* act, float* state, const size_t sizeof_state, const int action_len)
{
	float* py = 0;
    float max_act = -10000;
    int action_index = 0;

	if(!act->ctx){
		DEBUG_PRINT("Allocate ONNX context before calling %s\n", __FUNCTION__);
		return -1;
	}

	/*onnx_context_dump(ctx, 0);*/
    if(!act->input){
	    act->input = onnx_tensor_search(act->ctx, "input");
    }
    if(!act->output){
	    act->output = onnx_tensor_search(act->ctx, "output");
    }

	/*py = input->datas;
	for(i=0; i<sizeof(state)/sizeof(state[0]); i++){
		py[i] = state[i];
		printf("%f \n", py[i]);
	}*/	

	onnx_tensor_apply(act->input, (void *)state, sizeof_state);
	/*onnx_tensor_dump(input, 1);*/

	onnx_run(act->ctx);

	/*onnx_tensor_dump(output, 1);*/
	
	py = act->output->datas;
    max_act = py[0];
	for(int i=1; i<action_len; i++){
        if(py[i] > max_act){
            action_index = i;
            max_act = py[i];
        }
		/*printf("output[%d]=%f\n", i, py[i]);*/
	}
	
	/*onnx_tensor_softmax(output, results, len);
	for(int i=0; i<len; i++){
		printf("results[%d]=%f\n", i, results[i]);
	}*/
	/*onnx_context_free(ctx);*/

	return action_index;
}

