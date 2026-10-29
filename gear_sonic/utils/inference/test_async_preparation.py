import base64
import io
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
from PIL import Image
from gear_sonic.utils.inference.remote_policy_client import prepare_request,bind_reply

class PreparationTests(unittest.TestCase):
    def test_exact_image_and_raw_archive(self):
        image=np.random.default_rng(4).integers(0,256,(480,640,3),dtype=np.uint8)
        stamp=time.monotonic()
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'input.npz'
            request=dict(request_id=7,generation=3,observed_at=stamp,_input_path=str(path),
                         _input=dict(image=image,state=np.arange(31),observed_at=stamp))
            prepared=prepare_request(request)
            actual=np.array(Image.open(io.BytesIO(base64.b64decode(prepared['image_png']))))
            expected=np.array(Image.fromarray(image).convert('RGB').resize((224,224),Image.Resampling.BILINEAR))
            np.testing.assert_array_equal(actual,expected)
            with np.load(path) as archive:
                np.testing.assert_array_equal(archive['image'],image)
                self.assertEqual(float(archive['observed_at']),stamp)
            self.assertEqual(prepared['observed_at'],stamp)
            self.assertEqual(prepared['generation'],3)
            self.assertNotIn('_input',prepared)
            self.assertIn('_input',request)
            reply=bind_reply(prepared,dict(request_id=7,generation=3,ok=True),stamp+.02,stamp+1.)
            self.assertFalse(reply['ok'])  # Preparation never renews observation age.
    def test_encoded_request_compatibility(self):
        request=dict(image_png='encoded')
        self.assertIs(prepare_request(request),request)
if __name__=='__main__':unittest.main()
