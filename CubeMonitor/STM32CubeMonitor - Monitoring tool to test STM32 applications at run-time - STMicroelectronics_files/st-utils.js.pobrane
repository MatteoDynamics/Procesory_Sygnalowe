var STUtils = {};

STUtils.getLanguage = function (locale) {
    switch(locale) {
        case "zh":
            return "Chinese";
        case "ja":
            return "Japanese";
        case "ko":
            return "Korean";
        default:
            return "English";
    }
};

STUtils.isCookieEnabledInBrowser = function () {
    var isIEBrowser = false || !!document.documentMode;
    if(!isIEBrowser){
        return navigator.cookieEnabled;
    }
    return checkIfCookieEnabled ()
};

function checkIfCookieEnabled() {
    var testCookie = "testcookie" + new Date().getTime() + "=";
    document.cookie = testCookie;
    var isCookieEnabled = document.cookie.indexOf(testCookie) !== -1;
    //delete test cookie
    if(isCookieEnabled){
        document.cookie = testCookie + '=; expires=Thu, 01 Jan 1970 00:00:00 GMT;';
    }
    return isCookieEnabled;
}

STUtils.findUrlParameter = function (urlParam) {
    var pageURL = window.location.search.substring(1);
    var URLVariables = pageURL.split('&');
    for (var i = 0; i < URLVariables.length; i++) {
        var parameterName = URLVariables[i].split('=');
        if (parameterName[0] === urlParam) {
            return parameterName[1] === undefined ? '' : decodeURIComponent(parameterName[1]);
        }
    }
};

STUtils.isPfProduct = function (tabSlider = $('.js-st-tab__slider')) {
    return tabSlider.hasClass('exclude-product');
}

STUtils.isActiveTabLink = function (tabId) {
    if (STUtils.isPfProduct()) {
        var href = $('.js-st-tabs-tab[data-tab="' + tabId + '"]').attr('href');

        if (typeof href === 'undefined') {
            href = '#' + tabId;
        }

        return window.location.hash.indexOf(href) > -1;
    } else {
        return true;
    }
};

STUtils.setActiveTab = function(tabId){
    window.sliderActiveTab = tabId;
}

STUtils.getActiveTab = function(){
   return window.sliderActiveTab;
}

STUtils.isTabActive = function (tabId){
    return STUtils.getActiveTab() === tabId;
}

STUtils.getBrowserInfo = function () {
    var ua = navigator.userAgent,
        tem,
        M = ua.match(/(opera|chrome|safari|firefox|msie|trident|edge(?=\/))\/?\s*(\d+)(\.\d+)+/i) || [];
    if (/trident/i.test(M[1])) {
        tem = /\brv[ :]+(\d+)/g.exec(ua) || [];
        return {name: 'IE ', version: (tem[1] || '')};
    }
    if (M[1] === 'Chrome') {
        tem = ua.match(/\bOPR\/(\d+)/);
        if (tem != null) {
            return {name: 'Opera', version: tem[1]};
        }

        tem = ua.match(/\bEdge\/(\d+(.\d+)+)/i);
        if (tem != null) {
            return {name: 'Microsoft Edge', version: tem[1]};
        }
    }
    M = M[2] ? [M[1], M[0].substring(M[0].indexOf('/') + 1)] : [navigator.appName, navigator.appVersion, '-?'];
    if ((tem = ua.match(/version\/(\d+)/i)) != null) {
        M.splice(1, 1, tem[1]);
    }
    return {
        name: M[0],
        version: M[1]
    };
};

STUtils.getIpAddress = function () {
    var result = undefined;
    $.ajax({
        url: "//api.ipify.org?format=json",
        type: "GET",
        async: false,
        success: function (data) {
            result = (data && data.ip) ? data.ip : undefined;
        }
    });
    return result;
};

STUtils.getIpGeolocatedCountry = function (ipAddress) {
    var result = undefined;
    $.ajax({
        url: "/bin/st/geolocation/ip/country",
        type: "POST",
        data: {"clientIP": ipAddress},
        async: false,
        success: function (response) {
            result = (response === null || response === '') ? undefined : response;
        }
    });
    return result;
};

STUtils.getIpGeolocatedCountryCode = function (ipAddress) {
    var result = undefined;
    $.ajax({
        url: "/bin/st/geolocation/ip/country?code=true",
        type: "POST",
        data: {"clientIP": ipAddress},
        async: false,
        success: function (response) {
            result = (response === null || response === '') ? undefined : response;
        }
    });
    return result;
};

STUtils.isAppleDevice = function() {
    return navigator.userAgent.match(/(iPhone|iPod|iPad)/) !== null;
};

STUtils.localTimeConvertor = function (utcTime) {
    var localTime = new Date(utcTime);
    return localTime;
};

STUtils.getUserLocale = function(){
    if (navigator.languages != undefined)
        return navigator.languages[0];
    return navigator.language;
};

STUtils.jQuery = {
    resetInlineDisplay: function ($element) {
        var element = $element.get(0);

        if (element) {
            element.style.display = null;
        }
    }
};

// We use this method to generate unique sessions IDs
STUtils.generateGuid = function() {
    var sessionId = STUtils.getCookieByName("unomiSessionId");
    if(sessionId){
        return sessionId;
    }
    function s4() {
        return Math.floor((1 + Math.random()) * 0x10000)
            .toString(16)
            .substring(1);
    }
    sessionId = s4() + s4() + '-' + s4() + '-' + s4() + '-' +
        s4() + '-' + s4() + s4() + s4();

    STUtils.setCookieByName("unomiSessionId", sessionId);

    return sessionId;
};

STUtils.getCookieByName = function(name) {
    var value = ";" + document.cookie;
    var parts = value.split(";" + name + "=");
    if (parts.length === 2) {
        return parts.pop().split(';').shift();
    }
    else{
        //added the case where the element is not the first cookie of the page
        parts = value.split("; " + name + "=");
        if (parts.length === 2) {
            return parts.pop().split(';').shift();
        }
    }
};

STUtils.setCookieByName = function(name, value){
    document.cookie = encodeURIComponent(name) + "=" + encodeURIComponent(value) + "; path=/";
};

STUtils.deleteCookieByName = function(name){
    document.cookie = encodeURIComponent(name) + "=; path=/; Max-Age=-1;";
};

STUtils.getCsrfToken = function() {
    var request = new XMLHttpRequest();
    request.open('GET', '/libs/granite/csrf/token.json', false);
    request.send(null);
    if (request.status === 200) {
        return JSON.parse(request.responseText).token;
    }
};

STUtils.getScrollBarWidth = function() {
    var element = document.createElement("div");
    element.style.cssText = "overflow:scroll; visibility:hidden; position:absolute;";
    document.body.appendChild(element);
    var widthScrollBar = element.offsetWidth - element.clientWidth;
    element.remove();
    return widthScrollBar;
};

STUtils.toolsSoftwareFamilyIds = [
    "FM146",
    "FM2221",
    "FM147",
    "FM116",
    "FM2391",
    "FM2471"
];

STUtils.isSTMTools = function() {
    if(typeof productHierarchy !== "undefined" && productHierarchy.length){
        var productFamilyId = productHierarchy[productHierarchy.length - 1]
        return STUtils.toolsSoftwareFamilyIds.includes(productFamilyId);
    }
};

STUtils.isHiddenToolsAndSoftwareTab = function () {
    if (STUtils.isSTMTools()) {
        return true;
    }

    if (typeof digitalData.page !== 'undefined' && digitalData.page.category.primaryCategory === 'Product folder:solution') {
        return false;
    }

    var result = false;
    $.ajax({
        url: document.location.href.replace('.html','.page-settings.json'),
        type: "GET",
        async: false,
        success: function (data) {
            result = (data && data.hideToolsAndSoftwareTab) ? data.hideToolsAndSoftwareTab : false;
        }
    });
    return result;
};
