#%%
import face_recognition
import os
import numpy as np

image_dir = "test_images"

#%%

image1 = face_recognition.load_image_file(os.path.join(image_dir, "biden1.jpg"))

image2 = face_recognition.load_image_file(os.path.join(image_dir, "biden2.jpeg"))


# %%
def testfunc():
    locations = face_recognition.api.face_locations(
        image1,
        number_of_times_to_upsample=1, #* 1 is default, higher finds smaller faces. Tuning needed
        model="hog", #* Options: "hog" (faster on cpu, less accurate), "cnn" (faster on gpu, more accurate)
    )

    enc1 = face_recognition.api.face_encodings(image1, 
        known_face_locations=locations,
        model="small",
    )[0]
    #* Hog / small ~= 1.15 secs
    #* Cnn ~= 12 secs
    #* Large face encoding model doesn't seem substantially slower than small (both 600-100ms)
