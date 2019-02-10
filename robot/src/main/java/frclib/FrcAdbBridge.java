/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frclib;

import java.nio.file.Path;
import java.nio.file.Paths;

import trclib.TrcDbgTrace;

/**
 * AdbBridge interfaces to an Android Debug Bridge (adb) binary, which is needed
 * to communicate to Android devices over USB.
 *
 * adb binary provided by https://github.com/Spectrum3847/RIOdroid
 */
public class FrcAdbBridge
{
    private static final String moduleName = "FrcAdbBridge";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public final static String DEF_ADB_PATH = "/usr/bin/adb";
    private Path adbLocation;
    private static FrcAdbBridge instance = null;

    /**
     * Constructor: Create an instance of the object. Note that this is a private constructor. Caller should call
     * the getInstance() method to get the instance of ADB. There can only be one instance of ADB running.
     *
     * @param path specifies the file path where the ADB executable is located. If null, the default path is assumed.
     *             This parameter is ignored if the ADB instance has already been created in previous calls.
     */
    private FrcAdbBridge(String path) 
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        if (path != null)
        {
            adbLocation = Paths.get(path);
        }
        else
        {
            try
            {
                adbLocation = Paths.get(System.getenv("FRC_ADB_LOCATION"));
            }
            catch (Exception e)
            {
                adbLocation = Paths.get(DEF_ADB_PATH);
            }
        }
    }   //FrcAdbBridge

    /**
     * This method returns the global instance of ADB. If none exists yet, a new instance is created.
     *
     * @param path specifies the file path where the ADB executable is located. If null, the default path is assumed.
     *             This parameter is ignored if the ADB instance has already been created in previous calls.
     * @return global instance of ADB.
     */
    public static FrcAdbBridge getInstance(String path)
    {
        if (instance == null)
        {
            instance = new FrcAdbBridge(path);
        }

        return instance;
    }   //getInstance

    /**
     * This method executes an ADB command.
     * Command syntax:
     *  <cmd> [<global_options>] [<parameters>]
     * global options:
     *  -a         listen on all network interfaces, not just localhost
     *  -d         use USB device (error if multiple devices connected)
     *  -e         use TCP/IP device (error if multiple TCP/IP devices available)
     *  -s SERIAL  use device with given serial (overrides $ANDROID_SERIAL)
     *  -t ID      use device with given transport id
     *  -H         name of adb server host [default=localhost]
     *  -P         port of adb server [default=5037]
     *  -L SOCKET  listen on given socket for adb server [default=tcp:localhost:5037]
     * 
     * general commands:
     *  devices [-l]             list connected devices (-l for long output)
     *  help                     show this help message
     *  version                  show version num
     * 
     * networking:
     *  connect HOST[:PORT]      connect to a device via TCP/IP [default port=5555]
     *  disconnect [HOST[:PORT]]
     *      disconnect from given TCP/IP device [default port=5555], or all
     *  forward --list           list all forward socket connections
     *  forward [--no-rebind] LOCAL REMOTE
     *      forward socket connection using:
     *        tcp:<port> (<local> may be "tcp:0" to pick any open port)
     *        localabstract:<unix domain socket name>
     *        localreserved:<unix domain socket name>
     *        localfilesystem:<unix domain socket name>
     *        dev:<character device name>
     *        jdwp:<process pid> (remote only)
     *  forward --remove LOCAL   remove specific forward socket connection
     *  forward --remove-all     remove all forward socket connections
     *  ppp TTY [PARAMETER...]   run PPP over USB
     *  reverse --list           list all reverse socket connections from device
     *  reverse [--no-rebind] REMOTE LOCAL
     *      reverse socket connection using:
     *        tcp:<port> (<remote> may be "tcp:0" to pick any open port)
     *        localabstract:<unix domain socket name>
     *        localreserved:<unix domain socket name>
     *        localfilesystem:<unix domain socket name>
     *  reverse --remove REMOTE  remove specific reverse socket connection
     *  reverse --remove-all     remove all reverse socket connections from device
     * 
     * file transfer:
     *  push [--sync] LOCAL... REMOTE
     *      copy local files/directories to device
     *      --sync: only push files that are newer on the host than the device
     *  pull [-a] REMOTE... LOCAL
     *      copy files/dirs from device
     *      -a: preserve file timestamp and mode
     *  sync [all|data|odm|oem|product|system|vendor]
     *      sync a local build from $ANDROID_PRODUCT_OUT to the device (default all)
     *      -l: list but don't copy
     * 
     * shell:
     *  shell [-e ESCAPE] [-n] [-Tt] [-x] [COMMAND...]
     *      run remote shell command (interactive shell if no command given)
     *      -e: choose escape character, or "none"; default '~'
     *      -n: don't read from stdin
     *      -T: disable PTY allocation
     *      -t: force PTY allocation
     *      -x: disable remote exit codes and stdout/stderr separation
     *  emu COMMAND              run emulator console command
     * 
     * app installation:
     *  install [-lrtsdg] [--instant] PACKAGE
     *  install-multiple [-lrtsdpg] [--instant] PACKAGE...
     *      push package(s) to the device and install them
     *      -l: forward lock application
     *      -r: replace existing application
     *      -t: allow test packages
     *      -s: install application on sdcard
     *      -d: allow version code downgrade (debuggable packages only)
     *      -p: partial application install (install-multiple only)
     *      -g: grant all runtime permissions
     *      --instant: cause the app to be installed as an ephemeral install app
     *  uninstall [-k] PACKAGE
     *      remove this app package from the device
     *      '-k': keep the data and cache directories
     * 
     * backup/restore:
     *    to show usage run "adb shell bu help"
     * 
     * debugging:
     *  bugreport [PATH]
     *      write bugreport to given PATH [default=bugreport.zip];
     *      if PATH is a directory, the bug report is saved in that directory.
     *      devices that don't support zipped bug reports output to stdout.
     *  jdwp                     list pids of processes hosting a JDWP transport
     *  logcat                   show device log (logcat --help for more)
     * 
     * security:
     *  disable-verity           disable dm-verity checking on userdebug builds
     *  enable-verity            re-enable dm-verity checking on userdebug builds
     *  keygen FILE
     *      generate adb public/private key; private key stored in FILE,
     *      public key stored in FILE.pub (existing files overwritten)
     * 
     * scripting:
     *  wait-for[-TRANSPORT]-STATE
     *      wait for device to be in the given state
     *      State: device, recovery, sideload, or bootloader
     *      Transport: usb, local, or any [default=any]
     *  get-state                print offline | bootloader | device
     *  get-serialno             print <serial-number>
     *  get-devpath              print <device-path>
     *  remount                  remount partitions read-write
     *  reboot [bootloader|recovery|sideload|sideload-auto-reboot]
     *      reboot the device; defaults to booting system image but
     *      supports bootloader and recovery too. sideload reboots
     *      into recovery and automatically starts sideload mode,
     *      sideload-auto-reboot is the same but reboots after sideloading.
     *  sideload OTAPACKAGE      sideload the given full OTA package
     *  root                     restart adbd with root permissions
     *  unroot                   restart adbd without root permissions
     *  usb                      restart adb server listening on USB
     *  tcpip PORT               restart adb server listening on TCP on PORT
     * 
     * internal debugging:
     *  start-server             ensure that there is a server running
     *  kill-server              kill the server if it is running
     *  reconnect                kick connection from host side to force reconnect
     *  reconnect device         kick connection from device side to force reconnect
     *  reconnect offline        reset offline/unauthorized devices to force reconnect
     * 
     * environment variables:
     *  $ADB_TRACE
     *      comma-separated list of debug info to log:
     *      all,adb,sockets,packets,rwx,usb,sync,sysdeps,transport,jdwp
     *  $ADB_VENDOR_KEYS         colon-separated list of keys (files or directories)
     *  $ANDROID_SERIAL          serial number to connect to (see -s)
     *  $ANDROID_LOG_TAGS        tags to be used by logcat (see logcat --help)
     *
     * @param cmd specifies the ADB command and arguments.
     */
    public boolean runCommand(String cmd)
    {
        final String funcName = "runCommand";
        boolean success = true;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "cmd=%s", cmd);
        }

        try 
        {
            Runtime.getRuntime().exec(adbLocation + " " + cmd).waitFor();
        }
        catch (Exception e) 
        {
            TrcDbgTrace.getGlobalTracer().traceErr(funcName, "Failed to run Adb command <%s>: %s",
                cmd, e.getMessage());
            success = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
        }

        return success;
    }   //runCommand

    /**
     * This method starts ADB.
     */
    public void start() 
    {
        runCommand("start");
    }   //start

    /**
     * This method stops ADB.
     */
    public void stop() 
    {
        runCommand("stop");
    }   //stop

    public void restart()
    {
        stop();
        start();
    }   //restart

    public void portForward(int local_port, int remote_port) 
    {
        runCommand("forward tcp:" + local_port + " tcp:" + remote_port);
    }   //portForward

    public void reversePortForward(int remote_port, int local_port) 
    {
        runCommand("reverse tcp:" + remote_port + " tcp:" + local_port);
    }   //reversePortForward

}   //class FrcAdbBridge
