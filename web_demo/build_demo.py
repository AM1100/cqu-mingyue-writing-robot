"""Bundle the demonstration, libraries and CAD previews into one offline HTML."""
import base64
import json
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def data_url(path):
    path = Path(path)
    mime = "image/png" if path.suffix.lower() == ".png" else "image/jpeg"
    return "data:%s;base64,%s" % (mime, base64.b64encode(path.read_bytes()).decode())


def build():
    payload_path = ROOT / "cad_assets" / "web_data.json"
    data = json.loads(payload_path.read_text()) if payload_path.exists() else {
        "partCount": 25, "assemblyCount": 1, "parts": [],
        "basis": "原 CAD 预览保留原文件外形；运动演示为参数化重建，未求解原生装配配合。",
    }
    if not data["parts"]:
        for preview in sorted((ROOT / "cad_assets" / "previews").glob("*.png")):
            assembly = preview.stem == "荧光时钟装配体"
            data["parts"].append({
                "name": preview.stem + (".SLDASM" if assembly else ".SLDPRT"),
                "preview": str(preview), "assembly": assembly,
            })
    for part in data.get("parts", []):
        if part.get("preview") and not part.get("image"):
            preview = Path(part["preview"])
            if not preview.is_absolute():
                preview = ROOT / "cad_assets" / preview
            part["image"] = data_url(preview)
    scene_path = ROOT / "cad_assets" / "scene.json"
    if scene_path.exists():
        scene = json.loads(scene_path.read_text())
        for component in scene.get("components", []):
            vertices = component.pop("positions", None)
            if vertices is not None:
                component["positionsBase64"] = base64.b64encode(
                    struct.pack("<%sf" % len(vertices), *vertices)
                ).decode()
        data["scene"] = scene
        data["basis"] = "原 CAD 模式按原建模坐标展示静态缓存网格（原装仅含 1 根主动臂、两腕孔尚未闭合）；运动演示为独立参数化重建，未求解 SolidWorks 原生配合，不代表实物无碰撞。"
    manifest_path = ROOT / "cad_assets" / "manifest.json"
    if manifest_path.exists():
        summary = json.loads(manifest_path.read_text())["assembly_summary"]
        data["assemblySummary"] = {
            "topLevel": summary["top_level_references"],
            "leaves": summary["leaf_instances"],
            "missing": summary["missing_source_files_from_zip"],
            "unreferenced": summary["unreferenced_parts_in_zip"],
        }
    html = (ROOT / "demo_template.html").read_text()
    for token, path in [
        ("__THREE__", ROOT / "vendor" / "three.min.js"),
        ("__CONTROLS__", ROOT / "vendor" / "OrbitControls.js"),
        ("__KINEMATICS__", ROOT / "kinematics.js"),
    ]:
        html = html.replace(token, path.read_text().replace("</script", "<\\/script"))
    html = html.replace("__CAD_DATA__", json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c"))
    license_text = (ROOT / "vendor" / "THREE_LICENSE.txt").read_text()
    html = html.replace("<html lang=\"zh-CN\">", "<html lang=\"zh-CN\">\n<!-- Three.js and OrbitControls: " + license_text + " -->")
    output = ROOT / "writing_robot_demo.html"
    output.write_text(html)
    print(str(output), output.stat().st_size)
    return output


if __name__ == "__main__":
    build()
