window.HUFF.registerPage(function(globals) {
   "use strict";

   var SystemAPI = globals.SystemAPI;
   var UIXML = globals.UIXML;
   var CommonUI = globals.CommonUI;

   var isSystemBag = function(services) {
      return (services.indexOf(SystemAPI.ServiceType.LocalSystem) != -1);
   }

   var isNetAdapterBag = function(services) {
      return (services.indexOf(SystemAPI.ServiceType.LocalNetInterface) != -1);
   }

   function NetworkAdapter(bag) {
      var addressCount = bag.getProperty(SystemAPI.PropertyID.NetIPv4AddressCount).value;
      this.addresses = [];
      for (var i = 0; i < addressCount; ++i) {
         this.addresses.push(bag.getProperty(SystemAPI.PropertyID.NetIPv4Address0+i).value);
      }

      this.mode = bag.getProperty(SystemAPI.PropertyID.NetAdapterMode).value;
      this.type = bag.getProperty(SystemAPI.PropertyID.NetAdapterType).value;
   };

   function NetworkAdaptersProperty(adapters) {
      this.value = adapters;
      this.type = -1;
   }

   // We care only about the system bag. However, the system bag can have some
   // special properties, "scs_ipaddr" and "scs_status", that depend on the results
   // of other bags. This function merges the necessary information into the system
   // bag, then returns it.
   var mergeSyntheticPropertiesIntoSystemBag = function(bags) {
      var networkAdapters = [];
      var systemBag = null;

      // For network adapters, we have to pull out the address
      // and adapter types in order to populate the "scs_ipaddr" setting
      for (var b = 0, len = bags.length; b < len; b++) {
         var bag = bags[b];
         var bagServices = bag.getServices();

         if (isNetAdapterBag(bagServices)) {
            networkAdapters.push(new NetworkAdapter(bag));
         } else if (isSystemBag(bagServices)) {
            systemBag = bag;
         }
      }

      if (systemBag == null)
         return; // no system bag?

      // Inject our "scs_ipaddr" setting into the bag.
      systemBag.setProperty("scs_ipaddr", new NetworkAdaptersProperty(networkAdapters));

      return systemBag;
   }

	/* BEGIN - CTRE */
	var iCtreSplashIdx = 4;
	function SplashCtrePhoenixMessage() {
		//var canBusLink = "<a href=https://github.com/CrossTheRoadElec/Phoenix-diagnostics-client/blob/master/readme.md#phoenix-diagnostics-client>Phoenix Diagnostics Client</a>";
		var innerHTML = "<font color='darkgreen'>CAN Bus</font> features have been moved to <b>Phoenix Tuner</b>";

		var temp = document.getElementById("saveStatus");
		if(temp == null) {
			
		} else if(iCtreSplashIdx > 0) {
			temp.innerHTML = innerHTML + " " + iCtreSplashIdx; 
			--iCtreSplashIdx;
			setTimeout(SplashCtrePhoenixMessage, 1000);
		} else {
			temp.innerHTML = ""; 
		}
	}
	/* END - CTRE */

   var onLoad = function() {
      var filter = new SystemAPI.PropertyBag;
      filter.setProperty(SystemAPI.PropertyID.ItemIsDevice, new SystemAPI.BoolProperty(true));
      filter.setProperty(SystemAPI.PropertyID.ItemIsChassis, new SystemAPI.BoolProperty(true));
      filter.setProperty(SystemAPI.PropertyID.ItemHasService, new SystemAPI.BoolProperty(true));


      setTimeout(SplashCtrePhoenixMessage, 500); /* CTRE */

      SystemAPI.searchForItemsAndPropertiesFiltered(
         "", SystemAPI.FilterMode.Any, filter
      ).then(function (bagResults) {
         return mergeSyntheticPropertiesIntoSystemBag(bagResults.propertyBags);
      }).then(function (systemBag) {
         return UIXML.getDefinitionsForBag(systemBag);
      }).then(function (bagAndDefs) {
         return CommonUI.generateFromBagAndDefs(bagAndDefs);
      }).then(function (pageContext) {
         return pageContext.updateDelayedProperties();
      });

      return true;
   };

   var onUnload = function () {
      return true;
   };

   return {
      pageName: 'SystemConfig',
      title: {
         en: "System Configuration"
      },
      icon: "/HUFF/images/HardwareConfig.png",
      legacyHash: "#/NationalInstruments.Config.Hardware;component/Page.dyn.xaml",
      help: "https://wpilib.screenstepslive.com/s/currentCS/m/cs_hardware/l/262266-roborio-webdashboard",
      onLoad: onLoad,
      onUnload: onUnload
   };
});
