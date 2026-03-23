using System.IO;
using UnityEngine;

namespace RMPersistence
{
    /// <summary>
    /// 持久化路径解析工具。
    /// 数据文件存储在项目根目录下的 JSONData 文件夹中。
    /// </summary>
    public static class FilePathResolver
    {
        private const string RootFolderName = "JSONData";

        /// <summary>
        /// 获取 Unity 项目根目录（Assets 的父目录）。
        /// </summary>
        public static string GetProjectRoot()
        {
            return Directory.GetParent(Application.dataPath).FullName;
        }

        /// <summary>
        /// 获取指定套件的数据目录，自动创建。
        /// 路径格式：{ProjectRoot}/JSONData/{suiteName}/
        /// </summary>
        public static string GetSuiteDirectory(string suiteName)
        {
            string dir = Path.Combine(GetProjectRoot(), RootFolderName, suiteName);
            if (!Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            return dir;
        }

        /// <summary>
        /// 生成带时间戳的文件名。
        /// 格式：{suiteName}_{yyyyMMdd_HHmmss_fff}.json
        /// </summary>
        public static string GenerateFileName(string suiteName)
        {
            string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss_fff");
            return $"{suiteName}_{timestamp}.json";
        }

        /// <summary>
        /// 生成完整的文件路径（目录 + 文件名）。
        /// </summary>
        public static string GenerateFilePath(string suiteName)
        {
            string dir = GetSuiteDirectory(suiteName);
            string fileName = GenerateFileName(suiteName);
            return Path.Combine(dir, fileName);
        }
    }
}
