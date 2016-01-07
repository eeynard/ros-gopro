Vagrant.configure(2) do |config|
    config.vm.box = "ubuntu/trusty64"
    config.vm.hostname = "ros-master"

    config.vm.network :private_network, ip: '192.168.50.50'
    config.vm.synced_folder '.', '/vagrant', type: 'nfs'

    config.vm.network "public_network", bridge: "en1: Wi-Fi (AirPort)"

    config.vm.provider "virtualbox" do |vb|
        host = RbConfig::CONFIG['host_os']

        # Give VM 1/3 system memory & access to all cpu cores on the host
        if host =~ /darwin/
            cpus = `sysctl -n hw.ncpu`.to_i
            # sysctl returns Bytes and we need to convert to MB
            mem = `sysctl -n hw.memsize`.to_i / 1024 / 1024 / 4
        elsif host =~ /linux/
            cpus = `nproc`.to_i
            # meminfo shows KB and we need to convert to MB
            mem = `grep 'MemTotal' /proc/meminfo | sed -e 's/MemTotal://' -e 's/ kB//'`.to_i / 1024 / 4
        else # sorry Windows folks, I can't help you
            cpus = 2
            mem = 2048
        end

        if mem > 4096
            mem = 4096
        end

        vb.customize ["modifyvm", :id, "--memory", mem]
        vb.customize ["modifyvm", :id, "--cpus", cpus]
        vb.customize ["modifyvm", :id, "--natdnshostresolver1", "on"]
        vb.customize ["guestproperty", "set", :id, "--timesync-threshold", 10000 ]

        # Adds an USB to the virtual box
        vb.customize ["modifyvm", :id, "--usb", "on"]
    end

    config.vm.provision "shell", path: "install-ros.sh"
end

