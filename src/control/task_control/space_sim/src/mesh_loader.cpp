#include "space_sim_task_control/mesh_loader.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace space_sim_task_control
{
namespace
{

/// 顶点去重：STL 里同一个顶点会被每个相邻三角形各存一份
class MeshBuilder
{
public:
  void addTriangle(
    float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2)
  {
    shape_msgs::msg::MeshTriangle tri;
    tri.vertex_indices = {vertex(x0, y0, z0), vertex(x1, y1, z1), vertex(x2, y2, z2)};
    triangles_.push_back(tri);
  }

  size_t triangleCount() const { return triangles_.size(); }

  void fill(shape_msgs::msg::Mesh & out) const
  {
    out.triangles = triangles_;
    out.vertices.reserve(vertices_.size());
    for (const auto & v : vertices_) {
      geometry_msgs::msg::Point p;
      p.x = std::get<0>(v);
      p.y = std::get<1>(v);
      p.z = std::get<2>(v);
      out.vertices.push_back(p);
    }
  }

private:
  uint32_t vertex(float x, float y, float z)
  {
    const auto key = std::make_tuple(x, y, z);
    const auto it = index_.find(key);
    if (it != index_.end()) {
      return it->second;
    }
    const uint32_t id = static_cast<uint32_t>(vertices_.size());
    vertices_.push_back(key);
    index_[key] = id;
    return id;
  }

  std::map<std::tuple<float, float, float>, uint32_t> index_;
  std::vector<std::tuple<float, float, float>> vertices_;
  std::vector<shape_msgs::msg::MeshTriangle> triangles_;
};

bool looksBinaryStl(const std::vector<char> & buf)
{
  if (buf.size() < 84) {
    return false;
  }
  uint32_t n = 0;
  std::memcpy(&n, buf.data() + 80, sizeof(uint32_t));
  return buf.size() == 84 + static_cast<size_t>(n) * 50;
}

bool parseBinaryStl(const std::vector<char> & buf, MeshBuilder & mesh, std::string & err)
{
  uint32_t n = 0;
  std::memcpy(&n, buf.data() + 80, sizeof(uint32_t));
  const char * p = buf.data() + 84;
  for (uint32_t i = 0; i < n; ++i) {
    float v[12];
    std::memcpy(v, p, sizeof(v));  // 前 3 个是法向，忽略
    mesh.addTriangle(
      v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11]);
    p += 50;  // 12 float + 2 byte attribute
  }
  if (mesh.triangleCount() == 0) {
    err = "二进制 STL 里没有三角面";
    return false;
  }
  return true;
}

bool parseAsciiStl(const std::string & text, MeshBuilder & mesh, std::string & err)
{
  std::istringstream is(text);
  std::string tok;
  std::vector<float> vs;
  while (is >> tok) {
    if (tok != "vertex") {
      continue;
    }
    float x = 0, y = 0, z = 0;
    if (!(is >> x >> y >> z)) {
      break;
    }
    vs.push_back(x);
    vs.push_back(y);
    vs.push_back(z);
  }
  if (vs.size() % 9 != 0 || vs.empty()) {
    err = "ASCII STL 顶点数不是 9 的倍数（解析失败）";
    return false;
  }
  for (size_t i = 0; i + 8 < vs.size(); i += 9) {
    mesh.addTriangle(vs[i], vs[i + 1], vs[i + 2], vs[i + 3], vs[i + 4], vs[i + 5], vs[i + 6],
                     vs[i + 7], vs[i + 8]);
  }
  return true;
}

bool parseObj(const std::string & text, MeshBuilder & mesh, std::string & err)
{
  std::istringstream is(text);
  std::string line;
  std::vector<float> verts;  // x,y,z 连续存放
  size_t n = 0;
  while (std::getline(is, line)) {
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag == "v") {
      float x = 0, y = 0, z = 0;
      ls >> x >> y >> z;
      verts.push_back(x);
      verts.push_back(y);
      verts.push_back(z);
      n += 1;
    } else if (tag == "f") {
      // 支持 "f a b c"、"f a/b/c d/e/f g/h/i"、"f a//c ..."
      std::vector<uint32_t> idx;
      std::string item;
      while (ls >> item) {
        const size_t slash = item.find('/');
        const std::string first = slash == std::string::npos ? item : item.substr(0, slash);
        int i = 0;
        try {
          i = std::stoi(first);
        } catch (const std::exception &) {
          continue;
        }
        if (i < 0) {
          i = static_cast<int>(n) + i;  // OBJ 允许负索引（相对）
        }
        idx.push_back(static_cast<uint32_t>(i - 1));
      }
      for (size_t k = 2; k < idx.size(); ++k) {  // 多边形扇形三角化
        const auto get = [&](uint32_t id, int comp) {
          return verts[static_cast<size_t>(id) * 3 + static_cast<size_t>(comp)];
        };
        if (idx[0] * 3 + 2 >= verts.size() || idx[k - 1] * 3 + 2 >= verts.size() ||
          idx[k] * 3 + 2 >= verts.size())
        {
          err = "OBJ 面索引越界";
          return false;
        }
        mesh.addTriangle(
          get(idx[0], 0), get(idx[0], 1), get(idx[0], 2), get(idx[k - 1], 0), get(idx[k - 1], 1),
          get(idx[k - 1], 2), get(idx[k], 0), get(idx[k], 1), get(idx[k], 2));
      }
    }
  }
  if (mesh.triangleCount() == 0) {
    err = "OBJ 里没有可用的三角面";
    return false;
  }
  return true;
}

}  // namespace

bool loadMeshFile(const std::string & path, shape_msgs::msg::Mesh & out, std::string & err)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    err = "打不开网格文件: " + path;
    return false;
  }
  const std::vector<char> buf(
    (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (buf.empty()) {
    err = "网格文件为空: " + path;
    return false;
  }
  MeshBuilder mesh;
  bool ok = false;
  if (looksBinaryStl(buf)) {
    ok = parseBinaryStl(buf, mesh, err);
  } else {
    const std::string text(buf.begin(), buf.end());
    const bool ascii_stl = text.compare(0, 5, "solid") == 0;
    const bool is_obj = path.size() > 4 && path.compare(path.size() - 4, 4, ".obj") == 0;
    if (ascii_stl && !is_obj) {
      ok = parseAsciiStl(text, mesh, err);
    } else if (is_obj) {
      ok = parseObj(text, mesh, err);
    } else {
      // 兜底：先按 OBJ，再按 ASCII STL
      ok = parseObj(text, mesh, err) || parseAsciiStl(text, mesh, err);
    }
  }
  if (!ok) {
    return false;
  }
  mesh.fill(out);
  return true;
}

std::string resolvePackageFile(const std::string & package, const std::string & relative)
{
  try {
    return ament_index_cpp::get_package_share_directory(package) + "/" + relative;
  } catch (const std::exception &) {
    return {};
  }
}

}  // namespace space_sim_task_control
