# WA1系列

# 选配信息

||WA1\-S|WA1\-D|WA1\-E|
|---|---|---|---|
|头部相机|D435I|D435I|D435I|
|胸部相机|选配|D435I|D435I|
|腹部相机|无|无|无|
|手部相机|* [ ] 待确认|* [ ] 待确认|* [ ] 待确认|
|前激光|选配（MID360S）<br>VANJI\-716Mini（400L底盘自带）|选配（MID360S）<br>VANJI\-716Mini（400L底盘自带）|MID360S|
|后激光|VANJI\-716Mini（400L底盘自带）|VANJI\-716Mini（400L底盘自带）|VANJI\-716Mini（400L底盘自带）|
|力传感器|* [ ] 标配，型号待确认|* [ ] 标配，型号待确认|* [ ] 标配，型号待确认|
|手配置|F5D6\-P|F5D6\-LC|F5D6\-P|

# URDF坐标信息

## 头部相机

D08版本：

```XML
<link name="head_camera_d435i_link">
  <inertial>
      <origin
        xyz="0.00162772608237771 0.0016823059850623 0.0125024599401337"
        rpy="0 0 0" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <material name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
    </collision>
  </link>
  <joint name="head_camera_d435i_joint"
    type="fixed">
    <origin
      xyz="0.0651401529140153 0 0.112439207885702"
      rpy="0 0.261799387799031 0" />
    <parent
      link="HEAD" />
    <child
      link="head_camera_d435i_link" />
    <axis
      xyz="0 0 0" />
  </joint>
```



D11版本:

```XML
<link name="head_camera_d435i_link">
  <inertial>
      <origin
        xyz="0.00162772608237771 0.0016823059850623 0.0125024599401337"
        rpy="0 0 0" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <material name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
    </collision>
  </link>
  <joint name="head_camera_d435i_joint"
    type="fixed">
    <origin
      xyz="0.0651401529140169 0 0.115939207885699"
      rpy="0 0.261799387799031 0" />
    <parent
      link="HEAD" />
    <child
      link="head_camera_d435i_link" />
    <axis
      xyz="0 0 0" />
  </joint>
```



## 胸部相机

```XML
<link
    name="chest_camera_d435i_link">
    <inertial>
      <origin
        xyz="0.00162772608237771 0.0016823059850623 0.0125024599401337"
        rpy="0 0 0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <material
        name="">
        <color
          rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
    </collision>
  </link>
  <joint
    name="chest_camera_d435i_joint"
    type="fixed">
    <origin
      xyz="0.096161 0 0.40243"
      rpy="-0.00074686 0.52073 0" />
    <parent
      link="WAIST_PITCH" />
    <child
      link="chest_camera_d435i_link" />
    <axis
      xyz="0 0 0" />
  </joint>
```

## 前激光MID360S

```XML
<link
    name="front_lidar_mid360_link">
    <inertial>
      <origin
        xyz="0.000213085497967336 -6.08762795983875E-06 0.0221378677262921"
        rpy="0 0 0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
    </collision>
  </link>
  <joint
    name="front_lidar_mid360_joint"
    type="fixed">
    <origin
      xyz="0.38643 0 0.0078191"
      rpy="-0.087266 0 -1.5708" />
    <parent
      link="BASE" />
    <child
      link="front_lidar_mid360_link" />
    <axis
      xyz="0 0 0" />
  </joint>
```

## 前激光716mini

```XML
<link
    name="front_lidar_716mini_link">
    <inertial>
      <origin
        xyz="0.0286588997100717 1.36829306741637E-05 0.413501807197874"
        rpy="0 0 0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <material
        name="">
        <color
          rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
    </collision>
  </link>
  <joint
    name="front_lidar_716mini_joint"
    type="fixed">
    <origin
      xyz="0.3692 0 -0.12366"
      rpy="0 0 0" />
    <parent
      link="BASE" />
    <child
      link="front_lidar_716mini_link" />
    <axis
      xyz="0 0 0" />
  </joint>
```

## 后激光

```XML
<link
    name="back_lidar_716mini_link">
    <inertial>
      <origin
        xyz="-0.773863426783906 1.36837453232341E-05 -0.307301807197869"
        rpy="0 0 0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <material
        name="">
        <color
          rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
    </collision>
  </link>
  <joint
    name="back_lidar_716mini_joint"
    type="fixed">
    <origin
      xyz="-0.376 0 -0.017464"
      rpy="-3.1416 0 3.1416" />
    <parent
      link="BASE" />
    <child
      link="back_lidar_716mini_link" />
    <axis
      xyz="0 0 0" />
  </joint>
```

