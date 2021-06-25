.. _Arm_blocked:

=======================================
How to unblock the arm if it is blocked
=======================================

Maybe during your testing the arm has blocked and you don't know how to make it work again.

.. _Reasons:

Reasons of blocking
===================

There are 2 main reasons for the arm to be blocked. The first one is because the robot moved too fast, and the second one is because the robot is too close to a joint limit. Pushing the joints to their limits may put the arm into ``red`` state (Sorry, I did not succeed to go in the ``red`` state, so I put a beautiful image of a woodchuck instead).

.. image:: images/fci-architecture.png
    :align: center

.. _Solve:

Solving the blocking
====================

* The first thing to try when the robot is blocked is to push the black button and to pull it back.
* If this does not work, try to lock the joints and then unlock them on the :ref:`interface<Interface>`.
* If this does not work, try to move a little bit the arm of it's position and then execute a :ref:`move_to_start command<Command_test>`.
* If this does not work, the last solution is to shut down the controller and to :ref:`start it up again<Controller>`
