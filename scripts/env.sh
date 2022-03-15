if [[ $OSTYPE == 'darwin'* ]]; then
    export ANDROID_SDK_ROOT=$HOME/Library/Android/sdk
else
    echo "This script only works on Mac. It selfs the ANDROID_SDK_ROOT environment variable, so you should probably do this yourself on other platforms."
fi
