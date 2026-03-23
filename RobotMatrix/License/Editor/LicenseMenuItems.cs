#if UNITY_EDITOR
using UnityEditor;

namespace RMLicense.Editor
{
    public static class LicenseMenuItems
    {
        [MenuItem("RobotMatrix/License/Activate", false, 1000)]
        public static void OpenLicenseWindow()
        {
            LicenseEditorWindow.ShowWindow();
        }

        [MenuItem("RobotMatrix/License/Check Status", false, 1001)]
        public static void CheckStatus()
        {
            var status = RMLicense.LicenseGuard.Status;
            string message;
            switch (status)
            {
                case RMLicense.LicenseStatus.Valid:
                    message = $"License is valid.\n\nLicense: {RMLicense.LicenseManager.Instance.GetLicenseInfo()}\nDevice ID: {RMLicense.LicenseGuard.HardwareIdShort}";
                    break;
                case RMLicense.LicenseStatus.NotActivated:
                    message = "Not activated. Please activate your license.";
                    break;
                case RMLicense.LicenseStatus.HardwareMismatch:
                    message = "Hardware mismatch. The license was activated on a different machine.";
                    break;
                default:
                    message = $"License status: {status}";
                    break;
            }

            EditorUtility.DisplayDialog("RobotMatrix License Status", message, "OK");
        }

        [MenuItem("RobotMatrix/License/Deactivate", false, 1002)]
        public static void Deactivate()
        {
            if (EditorUtility.DisplayDialog("Deactivate License",
                "Are you sure you want to deactivate your license?", "Yes", "Cancel"))
            {
                RMLicense.LicenseManager.Instance.Deactivate();
                EditorUtility.DisplayDialog("License Deactivated",
                    "Your license has been deactivated. You will need to re-activate to use the plugin.", "OK");
            }
        }
    }
}
#endif
