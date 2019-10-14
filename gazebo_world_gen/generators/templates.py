# -*- coding: utf-8 -*-

PATH_TEMPLATE_START = """<?xml version="1.0" encoding="UTF-8"?>
<path name="{}">
<auto_start>{}</auto_start>
<delay_start>{}</delay_start>
<loop>{}</loop>
"""

PATH_TEMPLATE_END = "\n</path>"

WORLD_TEMPLATE_START = """<?xml version="1.0"?>
<sdf version='1.5'>
  <world name='default'>

    <gui>
      <camera name="user_camera">
        <pose>-7.67 1.63 2.5 0.00 0.143 -0.327</pose>
        <track_visual>
          <static>true</static>
          <use_model_frame>true</use_model_frame>
          <xyz>-3 0 1</xyz>
          <inherit_yaw>true</inherit_yaw>
        </track_visual>
      </camera>
    </gui>
    
    <spherical_coordinates>
      <latitude_deg>48.878922</latitude_deg>
      <longitude_deg>2.367782</longitude_deg>
    </spherical_coordinates>

    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <max_contacts>20</max_contacts>
      <gravity>0 0 -9.81</gravity>
      <magnetic_field>0.1062e-6 20.8038e-6 -43.2881e-6</magnetic_field>
      <ode>
        <solver>
          <type>world</type>
          <min_step_size>0.0001</min_step_size>
          <iters>50</iters>
          <precon_iters>0</precon_iters>
          <sor>1.4</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <atmosphere type="adiabatic">
      <temperature>298.15</temperature>
      <pressure>101325</pressure>
      <temperature_gradient>-0.0065</temperature_gradient>
    </atmosphere>

    <plugin name="fwman" filename="libsphinx_fwman.so">
      <spawn_point name="default">
        <pose>{}</pose>
      </spawn_point>
    </plugin>

    <!--*****************-->
    <!--Scene description-->
    <!--*****************-->
    <include>
      <uri>model://ground_plane</uri>
    </include>
"""                   

WORLD_TEMPLATE_END = """
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 400 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
"""