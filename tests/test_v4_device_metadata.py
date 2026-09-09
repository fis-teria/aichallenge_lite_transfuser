import torch
from aic_transfuser_lite.runtime.publisherless_shadow_v4 import inference_device_metadata

def test_records_actual_parameter_device_without_forward():
    model=torch.nn.Linear(1,1).cpu()
    def forbidden(*args, **kwargs):
        raise AssertionError('metadata must not run inference')
    model.forward=forbidden
    metadata=inference_device_metadata(model)
    assert metadata['model_device']=='cpu'
    assert metadata['gpu_name'] is None
    assert metadata['model_loaded_cuda_allocated_bytes']==0
    assert 'NOT_PEAK' in metadata['memory_scope']
