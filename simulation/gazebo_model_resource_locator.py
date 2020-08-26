import tesseract
import re
import traceback
import os

#resource locator class using GAZEBO_MODEL_PATH and model:// url
class GazeboModelResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(GazeboModelResourceLocator,self).__init__()
        model_env_path = os.environ["GAZEBO_MODEL_PATH"]
        self.model_paths = model_env_path.split(os.pathsep)
        assert len(self.model_paths) != 0, "No GAZEBO_MODEL_PATH specified!"
        for p in self.model_paths:
            assert os.path.isdir(p), "GAZEBO_MODEL_PATH directory does not exist: %s" % p

    def locateResource(self, url):
        try:
            url_match = re.match(r"^model:\/\/(\w+)\/(.+)$",url)
            if (url_match is None):
                assert False, "Invalid Gazebo model resource url %s" % url
            model_name = url_match.group(1)
            resource_path = os.path.normpath(url_match.group(2))

            for p in self.model_paths:
            
                fname = os.path.join(p, model_name, resource_path )
                if not os.path.isfile(fname):
                    continue
                with open(fname,'rb') as f:
                    resource_bytes = f.read()

                resource = tesseract.BytesResource(url, resource_bytes)

                return resource
            
            assert False, "Could not find requested resource %s" % url
        except:
            traceback.print_exc()
            return None