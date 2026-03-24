#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using RMLicense;
using System.Threading.Tasks;

namespace RMLicense.Editor
{
    public class LicenseEditorWindow : EditorWindow
    {
        private string _email = "";
        private string _licenseKey = "";
        private string _adminPassphrase = "";
        private bool _showAdminInput = false;
        private bool _showManualKeyInput = false;
        private string _statusMessage = "";
        private bool _isProcessing = false;
        private MessageType _messageType = MessageType.None;

        public static void ShowWindow()
        {
            var window = GetWindow<LicenseEditorWindow>(true, "RobotMatrix License", true);
            window.minSize = new Vector2(440, 400);
            window.maxSize = new Vector2(440, 420);
            window.ShowUtility();
        }

        private void OnEnable()
        {
        }

        private void OnGUI()
        {
            EditorGUILayout.Space(10);

            // Title
            var titleStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 16,
                alignment = TextAnchor.MiddleCenter
            };
            EditorGUILayout.LabelField("RobotMatrix License", titleStyle);
            EditorGUILayout.Space(6);

            // Status indicator
            DrawStatusBar();
            EditorGUILayout.Space(12);

            if (LicenseGuard.IsValid)
            {
                DrawLicensedUI();
            }
            else
            {
                DrawUnlicensedUI();
            }

            // Status message
            if (!string.IsNullOrEmpty(_statusMessage))
            {
                EditorGUILayout.Space(8);
                EditorGUILayout.HelpBox(_statusMessage, _messageType);
            }
        }

        private void DrawStatusBar()
        {
            var status = LicenseGuard.Status;
            string statusText;
            Color statusColor;

            switch (status)
            {
                case LicenseStatus.Valid:
                    statusText = "Licensed - " + LicenseManager.Instance.GetLicenseInfo();
                    statusColor = new Color(0.1f, 0.7f, 0.3f);
                    break;
                case LicenseStatus.HardwareMismatch:
                    statusText = "Hardware Mismatch";
                    statusColor = Color.red;
                    break;
                default:
                    statusText = "Not Activated";
                    statusColor = new Color(0.9f, 0.6f, 0.1f);
                    break;
            }

            var statusStyle = new GUIStyle(EditorStyles.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontStyle = FontStyle.Bold
            };
            var prevColor = GUI.color;
            GUI.color = statusColor;
            EditorGUILayout.LabelField(statusText, statusStyle);
            GUI.color = prevColor;
        }

        private void DrawUnlicensedUI()
        {
            EditorGUI.BeginDisabledGroup(_isProcessing);

            if (!_showManualKeyInput)
            {
                // ===== Default: Email request mode =====
                EditorGUILayout.HelpBox(
                    "Enter your email and click Request Access.\n" +
                    "Your device info will be sent automatically.\n" +
                    "You will receive a license key via email once approved.",
                    MessageType.Info);

                EditorGUILayout.Space(8);
                EditorGUILayout.LabelField("Email Address:");
                _email = EditorGUILayout.TextField(_email);

                EditorGUILayout.Space(6);

                if (GUILayout.Button(_isProcessing ? "Submitting..." : "Request Access", GUILayout.Height(32)))
                {
                    if (IsValidEmail(_email))
                    {
                        _ = RequestAccessAsync(_email.Trim());
                    }
                    else
                    {
                        _statusMessage = "Please enter a valid email address";
                        _messageType = MessageType.Warning;
                    }
                }

                EditorGUILayout.Space(12);

                // Separator
                EditorGUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();
                var sepStyle = new GUIStyle(EditorStyles.miniLabel) { alignment = TextAnchor.MiddleCenter };
                EditorGUILayout.LabelField("- or -", sepStyle, GUILayout.Width(40));
                GUILayout.FlexibleSpace();
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space(4);

                // Toggle to manual key input
                EditorGUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("I already have a license key", EditorStyles.linkLabel))
                {
                    _showManualKeyInput = true;
                    _statusMessage = "";
                }
                GUILayout.FlexibleSpace();
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                // ===== Manual key input mode =====
                EditorGUILayout.LabelField("License Key:");
                _licenseKey = EditorGUILayout.TextField(_licenseKey);
                EditorGUILayout.Space(4);

                if (GUILayout.Button(_isProcessing ? "Activating..." : "Activate", GUILayout.Height(30)))
                {
                    if (!string.IsNullOrWhiteSpace(_licenseKey))
                    {
                        _ = ActivateAsync(_licenseKey.Trim());
                    }
                    else
                    {
                        _statusMessage = "Please enter a license key";
                        _messageType = MessageType.Warning;
                    }
                }

                EditorGUILayout.Space(8);

                EditorGUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("Back to email request", EditorStyles.linkLabel))
                {
                    _showManualKeyInput = false;
                    _statusMessage = "";
                }
                GUILayout.FlexibleSpace();
                EditorGUILayout.EndHorizontal();
            }

            EditorGUILayout.Space(8);

            // Admin override (small unobtrusive button)
            EditorGUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Admin Override", EditorStyles.miniButton, GUILayout.Width(100)))
            {
                _showAdminInput = !_showAdminInput;
            }
            EditorGUILayout.EndHorizontal();

            if (_showAdminInput)
            {
                EditorGUILayout.Space(4);
                _adminPassphrase = EditorGUILayout.PasswordField("Passphrase:", _adminPassphrase);

                if (GUILayout.Button("Confirm", GUILayout.Height(22)))
                {
                    if (!string.IsNullOrWhiteSpace(_adminPassphrase))
                    {
                        var result = LicenseManager.Instance.ActivateAdmin(_adminPassphrase.Trim());
                        if (result.Success)
                        {
                            _statusMessage = "Admin override activation successful!";
                            _messageType = MessageType.Info;
                            _showAdminInput = false;
                            _adminPassphrase = "";
                        }
                        else
                        {
                            _statusMessage = result.Message;
                            _messageType = MessageType.Error;
                        }
                        Repaint();
                    }
                }
            }

            EditorGUI.EndDisabledGroup();
        }

        private void DrawLicensedUI()
        {
            EditorGUILayout.HelpBox("This machine is licensed. All RobotMatrix features are available.", MessageType.Info);
            EditorGUILayout.Space(8);

            if (GUILayout.Button("Deactivate License", GUILayout.Height(25)))
            {
                if (EditorUtility.DisplayDialog("Deactivate",
                    "Are you sure? You will need to re-activate.", "Deactivate", "Cancel"))
                {
                    LicenseManager.Instance.Deactivate();
                    _statusMessage = "License deactivated";
                    _messageType = MessageType.Info;
                    Repaint();
                }
            }
        }

        private async Task RequestAccessAsync(string email)
        {
            _isProcessing = true;
            _statusMessage = "Submitting request...";
            _messageType = MessageType.Info;
            Repaint();

            var result = await LicenseManager.Instance.RequestAccessAsync(email);

            _isProcessing = false;

            if (result.Success)
            {
                _statusMessage = result.Message;
                _messageType = MessageType.Info;
                _email = "";
            }
            else
            {
                _statusMessage = $"Failed: {result.Message}";
                _messageType = MessageType.Error;
            }

            Repaint();
        }

        private async Task ActivateAsync(string key)
        {
            _isProcessing = true;
            _statusMessage = "Activating...";
            _messageType = MessageType.Info;
            Repaint();

            var result = await LicenseManager.Instance.ActivateAsync(key);

            _isProcessing = false;

            if (result.Success)
            {
                _statusMessage = "Activation successful!";
                _messageType = MessageType.Info;
                _licenseKey = "";
            }
            else
            {
                _statusMessage = $"Activation failed: {result.Message}";
                _messageType = MessageType.Error;
            }

            Repaint();
        }

        private static bool IsValidEmail(string email)
        {
            if (string.IsNullOrWhiteSpace(email)) return false;
            int atIndex = email.IndexOf('@');
            if (atIndex <= 0) return false;
            int dotIndex = email.LastIndexOf('.');
            return dotIndex > atIndex + 1 && dotIndex < email.Length - 1;
        }
    }
}
#endif
