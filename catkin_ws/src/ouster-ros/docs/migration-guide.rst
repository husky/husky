.. title:: Ouster-ROS Driver Migration Guide


===================================
Migrating from 20220608 to 20220826
===================================
The ``20220826`` release brought several improvements to the **ouster-ros** driver, however, these
improvements brought also several breaking changes. This guide summarizes these change and how to
mitigate each one.

Launch Files:
=============
The ``ouster.launch`` has been split into three functionally different launch files:
``sensor.launch``, ``replay.launch``, and ``record.launch``. Each of these launch files only accepts
parameters that are relevant to their functional use. For example, the ``sensor_hostname`` argument
is only valid and required when using the ``sensor.launch`` and ``record.launch`` launch files.
The ``metadata`` argument however is no longer required when connecting to the sensor in live mode.
It is still required when using ``record.launch`` and ``replay.launch``. Refer to the `main
documentation <./doc/index.rst>`_ for more details on the different cases of usage.

Topics
======

Topics Renaming
---------------

Rather than having each topic published by **ouster-ros** be prefixed with the name of the ros node
that publishes it, the topic names of all ros nodes that compose the **ouster-ros** driver have been
combined under a single namespace. Thus all topics would appear prefixed by the new namespace.

If a user wishes to maintain the old topic names then they can achieve that by utilizing the
``<remap>`` tag in ros launch files. For example, let's say we want to remap the three topics
published by ``os_cloud_node`` to their old names when connecting to a sensor through the
``sensor.launch`` file. To do so, we need to edit the ``sensor.launch`` file and add the three
following remap tags right before any node is instantiated (i.e. the ``<remap>`` tag defintions
should precede any ``<node>`` tags in the launch file)::

    <remap from="/$(arg ouster_ns)/imu" to="/os_cloud_node/imu"/>
    <remap from="/$(arg ouster_ns)/points" to="/os_cloud_node/points"/>
    <remap from="/$(arg ouster_ns)/points2" to="/os_cloud_node/points2"/>

Same thing needs to happen if we want to remap the topic names that are published by ``img_node``.
This is shown below as well::

    <remap from="/$(arg ouster_ns)/nearir_image" to="/img_node/nearir_image"/>
    <remap from="/$(arg ouster_ns)/range_image" to="/img_node/range_image"/>
    <remap from="/$(arg ouster_ns)/reflec_image" to="/img_node/reflec_image"/>
    <remap from="/$(arg ouster_ns)/signal_image" to="/img_node/signal_image"/>

Topics Dropped
--------------

Additionally, the following two topic have been dropped::

    /os1_node/imu_packets
    /os1_node/lidar_packets

These two topics are a duplicates of the ``/os_node/imu_packets`` and ``/os_node/lidar_packets``,
which both have been renamed to  ``/ouster/imu_packets`` and ``/ouster/lidar_packets`` respectively.
If you rely on these specific topics names in your integration then you could easily follow same
steps described in the `Topics Renaming`_ section.

Services
========
This update brought two additional ros services and changed the name of the only ros service that
existed prior to this release. The service name was ``/os_config`` and when it is invoked it would
retrive the sensor metadata. This service was also combined under the unifed namespace ``ouster``
and was renamed to ``get_metadata`` to match its actual functionality. In case the user wish to
maintain the old name they can achieve so in similar manner that was described in the previous
section. In short edit the launch files and add the following ```<remap>`` tag prior to the 
instantiation of any of the three nodes::

    <remap from="/$(arg ouster_ns)/get_metadata" to="/os_config"/>
