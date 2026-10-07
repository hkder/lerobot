// SO101.exe: starts the SO-101 operator console (so101/app/main.py) without a console window.
// Build with so101/app/build_exe.cmd. The exe stays in so101/ and always runs the repo's current code.
using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Windows.Forms;

[assembly: System.Reflection.AssemblyTitle("SO-101 Operator Console")]
[assembly: System.Reflection.AssemblyProduct("SO-101 Operator Console")]
[assembly: System.Reflection.AssemblyVersion("1.0.0.0")]

static class Launcher
{
    const string Extras = "--extra dataset --extra feetech --extra viz --extra async --extra hardware";

    [STAThread]
    static int Main()
    {
        bool first;
        using (var mutex = new Mutex(true, "SO101OperatorConsole", out first))
        {
            if (!first)
                return Fail("The SO-101 console is already open. Two copies would fight over the arms.");

            string repo = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, ".."));
            string app = Path.Combine(repo, "so101", "app", "main.py");
            if (!File.Exists(app))
                return Fail("Can't find " + app + ".\nKeep SO101.exe in the so101 folder of the lerobot repo.");

            string uv = FindUv();
            if (uv == null)
                return Fail("uv is not installed.\nInstall it from https://docs.astral.sh/uv/ and try again.");

            if (!Directory.Exists(Path.Combine(repo, ".venv")))
            {
                // First run on this machine: install LeRobot, in a visible window so progress shows.
                var sync = Process.Start(new ProcessStartInfo(uv, "sync --locked --python 3.12 " + Extras)
                {
                    WorkingDirectory = repo,
                    UseShellExecute = false,
                });
                sync.WaitForExit();
                if (sync.ExitCode != 0)
                    return Fail("Installing LeRobot (uv sync) failed. Run it in a terminal in " + repo + " to see why.");
            }

            // PySide6 lives in the project venv so the app starts in about a second
            // (`uv run --with` rebuilt a temporary env on every launch: 8-13 s). `uv sync` removes it, so check.
            if (!Directory.Exists(Path.Combine(repo, ".venv", "Lib", "site-packages", "PySide6")))
            {
                var pip = Process.Start(new ProcessStartInfo(uv, "pip install --python .venv PySide6")
                {
                    WorkingDirectory = repo,
                    UseShellExecute = false,
                });
                pip.WaitForExit();
                if (pip.ExitCode != 0)
                    return Fail("Installing PySide6 failed. Run `uv pip install --python .venv PySide6` in " + repo + ".");
            }

            var psi = new ProcessStartInfo(Path.Combine(repo, ".venv", "Scripts", "pythonw.exe"), "so101\\app\\main.py")
            {
                WorkingDirectory = repo,
                UseShellExecute = false,
                CreateNoWindow = true,
                RedirectStandardError = true,
            };
            var proc = Process.Start(psi);
            string errors = proc.StandardError.ReadToEnd();  // returns when the app closes
            proc.WaitForExit();
            if (proc.ExitCode != 0)
                return Fail("The console stopped with an error:\n\n" + Tail(errors, 15) +
                            "\n\nFull log: " + Path.Combine(repo, "outputs", "so101_app.log"));
            return 0;
        }
    }

    static string FindUv()
    {
        foreach (string dir in (Environment.GetEnvironmentVariable("PATH") ?? "").Split(';'))
        {
            try
            {
                string candidate = Path.Combine(dir.Trim(), "uv.exe");
                if (File.Exists(candidate)) return candidate;
            }
            catch (ArgumentException) { }
        }
        string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
        string local = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
        foreach (string candidate in new[] {
            Path.Combine(home, ".local", "bin", "uv.exe"),
            Path.Combine(home, ".cargo", "bin", "uv.exe"),
            Path.Combine(local, "Microsoft", "WinGet", "Links", "uv.exe"),
        })
        {
            if (File.Exists(candidate)) return candidate;
        }
        return null;
    }

    static string Tail(string text, int lines)
    {
        string[] all = (text ?? "").Trim().Split('\n');
        int start = Math.Max(0, all.Length - lines);
        return string.Join("\n", all, start, all.Length - start);
    }

    static int Fail(string message)
    {
        MessageBox.Show(message, "SO-101 Operator Console", MessageBoxButtons.OK, MessageBoxIcon.Error);
        return 1;
    }
}
