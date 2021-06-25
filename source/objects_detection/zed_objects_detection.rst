Objects detection with ZED2 Camera
==================================

Download example code
---------------------
If you have not done it yet, the example code for the zed camera can be cloned with:

.. code-block:: bash

        git clone https://github.com/stereolabs/zed-examples.git

Code explanation
----------------
You can find `the tutorial for 3D objects detection on Stereolab website <https://www.stereolabs.com/docs/tutorials/3d-object-detection/>`_

Run python code
---------------
| The script is located in path-to-ros-example/object detection/image\ viewer/python
| Run python script as it is explained in :ref:`this section <run_python_script>`.
| The first time you run it it will download the AI model, this can take some time.

Change the code
---------------

* Detect other objects

The code here can only detect person, if you want to detect other objects change the line 73:

.. code-block:: python

        obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]  # Only detect Persons

Add vehicule object in the list:

.. code-block:: python

        obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON, sl.OBJECT_CLASS.VEHICLE]  # detect persons and vehicules

So you can detect persons and vehiculs

You can find `the list of available object class on stereolabs website <https://www.stereolabs.com/docs/api/group__Object__group.html>`_

* Print detected objects information

| We changed the code to print the objects detected every 1 second with its id and position:

.. code-block:: python

    import time
    timer=time.time()
    while viewer.is_available():
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Retrieve objects
            zed.retrieve_objects(objects, obj_runtime_param)
            # print detected objects information
            if time.time()-timer>1:
                print([[obj.id, obj.label, obj.position] for obj in objects.object_list])
                timer = time.time()
            # Update GL view
            viewer.update_view(image, objects)

You can find the `list of objects information that can be print on stereolabs website <https://www.stereolabs.com/docs/api/classsl_1_1ObjectData.html>`_