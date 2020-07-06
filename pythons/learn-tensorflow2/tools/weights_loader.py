"""
@author: xiaofeng.qi
"""


class WeightsLoader:
    def __init__(self, reference_full_path):
        import os
        dot = '.'
        ref_name = os.path.basename(reference_full_path)
        self._fileName = ref_name[:ref_name.rindex(dot)] + dot + 'h5'

    def save_weights(self, model):
        model.save_weights(self._fileName)

    def load_weights(self, model):
        model.load_weights(self._fileName)

    def has_weights(self):
        import os
        return os.path.exists(self._fileName)
