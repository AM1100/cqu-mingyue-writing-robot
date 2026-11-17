# 原始建模文件

solidworks/保持用户原包全部25个SLDPRT和1个SLDASM的原名及内容，源模型未改动。

原主装配只引用1根主动臂，两腕孔未闭合；另有14个零件未被引用。原包缺少以下11项原生依赖，未补造或冒称完成配合：

- Part10^servo motor assembley10.SLDPRT
- Part16^servo motor assembley4.SLDPRT
- Part6^servo motor assembley16.SLDPRT
- Part7^servo motor assembley14.SLDPRT
- Part8^servo motor assembley12.SLDPRT
- feedback potentiometer18.SLDPRT
- lower cover22.SLDPRT
- motor20.SLDPRT
- upper cover8.SLDPRT
- 舵机装配体.SLDASM
- 舵机转动臂.SLDPRT

主装配缓存可恢复44个实例、72758个三角面，用于HTML原CAD查看；缓存数据在 ../web_demo/cad_assets/scene.json，装配树、尺寸和零件预览也在同目录。网页完整五连杆运动是独立参数化重建，源代码在web_demo/demo_template.html与kinematics.js，不是另一套补齐配合的SolidWorks总装。
