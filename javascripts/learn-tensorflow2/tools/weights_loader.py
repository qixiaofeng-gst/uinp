"""
@author: xiaofeng.qi
"""

class WeightsLoader:
    def __init__(self, referenceFullPath):
        import os
        dot = '.'
        refName = os.path.basename(referenceFullPath)
        self._fileName = refName[:refName.rindex(dot)] + dot + 'h5'

    def saveWeights(self, model):
        model.save_weights(self._fileName)

    def loadWeights(self, model):
        model.load_weights(self._fileName)

    def hasWeights(self):
        import os
        return os.path.exists(self._fileName)
