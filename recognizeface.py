#%%
import face_recognition
import os
import numpy as np

image_dir = "test_images"

#%%

image1 = face_recognition.load_image_file(os.path.join(image_dir, "biden1.jpg"))

image2 = face_recognition.load_image_file(os.path.join(image_dir, "biden2.jpeg"))


# %%

enc1 = face_recognition.api.face_encodings(image1)[0]
enc2 = face_recognition.api.face_encodings(image2)[0]

# %%

print(face_recognition.api.face_distance([enc1], enc2))

# %%
