%define _unpackaged_files_terminate_build 1

Name: bootterm
Version: 0.4.0
License: %mit
Release: alt1
Summary: Simple terminal designed to ease connection to ephemeral serial ports
Group: Other
URL: https://github.com/wtarreau/bootterm

Source0: %name-%version.tar

BuildRequires(pre): rpm-build-licenses

%description
Bootterm is a simple, reliable and powerful terminal designed to ease
connection to ephemeral serial ports as found on various SBCs, and
typically USB-based ones.

%prep
%setup -q

%build
%make

%install
make PREFIX=%buildroot/usr install

%files
%_bindir/bt

%changelog
* Fri Oct 01 2021 Igor Chudov <nir@altlinux.org> 0.4.0-alt1
- Initial release

