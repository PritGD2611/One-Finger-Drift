#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using System.IO;

// Pre-build cleanup for Android builds.
// Attempts to remove read-only attributes and delete the Unity-generated gradle/IL2CPP intermediate
// folder that sometimes gets locked (Library/Bee/Android/Prj...). This is a last-resort workaround
// to recover from java.nio.file.AccessDeniedException during build.
// Note: this is a mitigation. If the folder is locked by another process (antivirus, adb, gradle, etc.)
// you should close the locking process or restart the Editor/PC.

public class PreBuildCleanup : IPreprocessBuildWithReport
{
    // Run early
    public int callbackOrder => 0;

    public void OnPreprocessBuild(BuildReport report)
    {
        // Only run for Android builds
        if (report.summary.platform != BuildTarget.Android)
            return;

        string projectDir = Directory.GetParent(Application.dataPath).FullName;

        // Path to the problematic folder inside Library
        string beePrjPath = Path.Combine(projectDir, "Library", "Bee", "Android", "Prj");

        try
        {
            if (Directory.Exists(beePrjPath))
            {
                Debug.Log("PreBuildCleanup: Attempting to clear attributes and delete: " + beePrjPath);
                MakeWritableRecursive(beePrjPath);
                Directory.Delete(beePrjPath, true);
                Debug.Log("PreBuildCleanup: Deleted folder: " + beePrjPath);
            }
            else
            {
                Debug.Log("PreBuildCleanup: Folder not found, nothing to clean: " + beePrjPath);
            }
        }
        catch (System.Exception ex)
        {
            // If deletion fails, log a helpful message and continue. The build will likely still fail,
            // but the message will point to the real cause (locked file / permissions).
            Debug.LogWarning($"PreBuildCleanup: Could not remove '{beePrjPath}': {ex.Message}\nFull: {ex}");
        }
    }

    static void MakeWritableRecursive(string path)
    {
        // Remove read-only attribute on files and directories
        try
        {
            var di = new DirectoryInfo(path);
            foreach (var dir in di.GetDirectories("*", SearchOption.AllDirectories))
            {
                try { dir.Attributes = FileAttributes.Normal; } catch { }
            }

            foreach (var file in di.GetFiles("*", SearchOption.AllDirectories))
            {
                try { file.Attributes = FileAttributes.Normal; } catch { }
            }

            // Also set the root
            try { di.Attributes = FileAttributes.Normal; } catch { }
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning("PreBuildCleanup: MakeWritableRecursive failed: " + ex.Message);
        }
    }

    // Optional menu to run cleanup manually from Editor
    [MenuItem("Build/PreBuild Cleanup (Android)")]
    public static void RunManualCleanup()
    {
        string projectDir = Directory.GetParent(Application.dataPath).FullName;
        string beePrjPath = Path.Combine(projectDir, "Library", "Bee", "Android", "Prj");

        if (Directory.Exists(beePrjPath))
        {
            MakeWritableRecursive(beePrjPath);
            try
            {
                Directory.Delete(beePrjPath, true);
                Debug.Log("PreBuildCleanup: manual delete succeeded: " + beePrjPath);
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning("PreBuildCleanup manual delete failed: " + ex.Message);
            }
        }
        else
        {
            Debug.Log("PreBuildCleanup: nothing to delete: " + beePrjPath);
        }
    }
}
#endif
