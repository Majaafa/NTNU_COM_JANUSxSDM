ABSTRACT

JANUS is a physical layer coding open-source promoted and developed by CMRE.
It is freely distributed under the GNU General Public License version 3.
The two primary purposes for JANUS are to announce the presence of a node and to
establish the initial contact between dissimilar nodes, establishing this way an
UW “lingua franca”.

BUILDING and INSTALLING

Example code to build the code in the directory 'build' inside the JANUS source
tree.  In this example the produced program will be installed in the directory
local_install also located inside the source tree.
Binaries will be compiled with debug option and allowed the prints for the
plug-ins code.

mkdir build
mkdir local-install
cd build
cmake -DCMAKE_INSTALL_PREFIX=../local-install -DCMAKE_BUILD_TYPE=Debug         \
-DJANUS_PLUGINS_DEBUG=1 ..                                                    &&
make -j8                                                                      &&
make install

In order to use floating point single precision where possible add cmake option:
  -DJANUS_REAL_SINGLE=1
To use single precision only in the fftw3 library (used in the base-bander) use:
  -DJANUS_FFTW_SINGLE=1

RUNNING EXAMPLES

Generation a base packet wav file named /tmp/janus.wav with sampling
rate of 48kHz.

./local-install/bin/janus-tx --verbose 2                                       \
  --pset-file ./local-install/share/janus/etc/parameter_sets.csv               \
  --pset-id 1 --stream-driver 'wav' --stream-driver-args '/tmp/janus.wav'      \
  --stream-fs 48000

Running the decoder using alsa using the first audio device:

 ./local-install/bin/janus-rx --verbose 2                                      \
  --pset-file ./local-install/share/janus/etc/parameter_sets.csv --pset-id 1   \
  --stream-driver 'alsa' --stream-driver-args 'hw:0' --stream-channels 2       \
  --stream-channel 0 --stream-fs 48000

Using a configuration file but imposing a different option:

./local-install/bin/janus-rx                                                   \
  --config-file ./local-install/share/janus/etc/janus.conf  --stream-chnnel 1

where the file janus.conf might look like:

--verbose 2
--pset-file ./local-install/share/janus/etc/parameter_sets.csv
--pset-id 1
--stream-driver alsa
#--stream-driver-args pulse
--stream-driver-args hw:Pro70789091,0
--stream-fs 48000
--stream-format S24_3LE
--stream-channels 2
--stream-channel 0

In this configuration is used a spcific device (/proc/alsa/cards).

In order to use the plug-ins:

./local-install/bin/janus-tx                                                   \
  --config-file ./local-install/share/janus/etc/janus.conf                     \
  --stream-channel 0 --packet-class-id 2 --packet-app-type 8                   \
  --packet-app-fields "Node_Type=2,Latitude=49.5,Longitude=9.5,Depth=10,Speed=1,Heading=90,Navigation_Status=0,Number_Contacts=0"
